package commands;

import com.kuka.device.common.JointPosition;
import com.kuka.geometry.Frame;
import com.kuka.geometry.LocalFrame;
import com.kuka.geometry.Tool;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Vector3D;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.motion.IMotionContainer;
import com.kuka.servoing.api.common.IServoMotion;
import com.kuka.servoing.api.common.IServoingCapability;
import com.kuka.servoing.api.smartservo.ISmartServo;
import com.kuka.servoing.api.smartservo.ISmartServoRuntime;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import functions.FriManager;
import functions.MotionManager;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import protocols.DefualtProtocol;
import protocols.ProtocolResult;
import units.AbstractCommand;
import units.AbstractCommandEx;

public class StartServo extends AbstractCommandEx {
  @Inject private LBRMed robot;
  @Inject private ITaskLogger logger;
  @Inject private World world;
  @Inject private IServoingCapability servoingCapability;
  @Inject private MotionManager motionManager;
  @Inject private FriManager friManager;
  
  private ISmartServoRuntime m_theSmartServoRuntime;
  private boolean m_continue = true;
  private boolean m_debug = false;
  private Tool tool;
  private int m_debugSteps =0;
  
  @Override
  public String GetNameString() {
    return "StartServo";
  }

  @Override
  public AbstractCommand CreateCommand() {
    return new StartServo();
  }

  @Override
  public ProtocolResult Execute(Object protocol) {
    DefualtProtocol p = (DefualtProtocol) protocol;
    tool = (Tool) robot.findObject("tool");
 // Create a SmartServo motion to the current position
    // By default the servo will stay active 30 seconds to wait for a new target destination
    // This timing can be configured by the user
    JointPosition initialPosition = robot.getCurrentJointPosition();
    ISmartServo smartServoMotion = servoingCapability.createSmartServoMotion(initialPosition);
    
    Frame initialPos = robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame());

    smartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

    // Validate the load configuration at the current position
    performLoadValidation(smartServoMotion);
    
    logger.info("Starting SmartServo motion in position control mode");
    IMotionContainer mc = tool.moveAsync(smartServoMotion);
    motionManager.setMotionContainer(mc);

    logger.info("Get the runtime of the SmartServo motion");
    m_theSmartServoRuntime = smartServoMotion.getRuntime(500, TimeUnit.MILLISECONDS);
    
    //start fri session so that friDynamicFrame will be update
    friManager.startFriSession();

    try {
      while (m_continue) {
        //get visual servoing frame(processed ndi data) from friManager
        //Frame destFrame = friManager.GetFriDynamicFrame();
        Frame offset = friManager.GetFriDynamicFrame();
        Frame destFrame = initialPos;
        destFrame.translate(offset.getTransformationFromParent().getTranslation());
        // Update the runtime information with the latest robot status
        m_theSmartServoRuntime.updateWithRealtimeSystem();

        m_theSmartServoRuntime.setDestination(destFrame);

        printDebugData();
      }
    } catch (Exception e) {
      logger.error(e.toString());
      e.printStackTrace();
    }

    // Print statistics and parameters of the motion
    logger.info("Displaying final states after loop ");
    logger.info(getClass().getName() + m_theSmartServoRuntime.toString());

    // Stop the motion
    logger.info("Stop the SmartServo motion");
    m_theSmartServoRuntime.stopMotion();
    
    ProtocolResult ret = new ProtocolResult();
    ret.setOperateType(p.getOperateType());    
    ret.setResultMsg("Servo end");
    ret.setResultCode(0);
    return ret;
  }
  
  public void performLoadValidation(IServoMotion servoMotion) {
    try {
      servoMotion.validateForImpedanceMode(tool);

      if (!servoMotion.isValidatedForImpedanceMode()) {
        logger.info("Validation of load data failed - correct your mass property settings");
        logger.info("SmartServo will be available for position controlled mode only, until"
            + " validation is performed");
      }
    } catch (IllegalStateException e) {
      logger.info("Omitting validation failure for this example\n" + e.getMessage());
    }
  }

  /**
   * Print current system information, if the debug printing is enabled.
   */
  public void printDebugData() {
    // ////////////////////////////////////////////////////////////
    if (m_debug) {
      m_debugSteps++;
      // Get the measured cartesian pose (robot root reference)
      Frame msrPose =
          m_theSmartServoRuntime
              .getCurrentCartesianPosition(tool.getDefaultMotionFrame());

      logger
          .info("Current joint destination " + m_theSmartServoRuntime.getCurrentJointDestination());
      logger.info(
          "LBR position "
              + robot.getCurrentCartesianPosition(tool.getDefaultMotionFrame(), robot.getRootFrame()));
      logger.info("Measured cartesian pose from runtime " + msrPose);

      if ((m_debugSteps % 100) == 0) {
        // Some internal values, which can be displayed
        logger
            .info("ServoRuntime info - step " + m_debugSteps + m_theSmartServoRuntime.toString());
      }
    }
  }
}
