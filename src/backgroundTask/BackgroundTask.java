package backgroundTask;

import com.kuka.device.ForceData;
import com.kuka.device.common.JointPosition;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.math.geometry.Transformation;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPIBackgroundTask;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.task.ITaskLogger;
import java.io.IOException;
import java.net.SocketException;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import protocols.GsonUtil;
import protocols.robotInformation;
import protocols.trackingFrame;
/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */

public class BackgroundTask extends RoboticsAPICyclicBackgroundTask {
  @Inject
  private LBRMed robot;
  @Inject 
  private ITaskLogger logger;
  private Tool tool;
  private UDPSocketForBackground soc;
  
  @Override
  public void initialize() {
    // initialize your task here
    initializeCyclic(0, 33, TimeUnit.MILLISECONDS,
        CycleBehavior.BEST_EFFORT);
    
    try {
      soc = new UDPSocketForBackground("172.31.1.148", 30003);
    } catch (SocketException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  @Override
  public void runCyclic() {   
    //execution starts here
    robotInformation info = new robotInformation();
    
    //get joint position
    JointPosition jointPosition = robot.getCurrentJointPosition();  
    info.joints = jointPosition.toArray();
    
    //get force & torque
    ForceData forceData = robot.getExternalForceTorque(robot.getFlange());
    info.forcetorque[0] = forceData.getForce().getX();
    info.forcetorque[1] = forceData.getForce().getY();
    info.forcetorque[2] = forceData.getForce().getZ();
    info.forcetorque[3] = forceData.getTorque().getX();
    info.forcetorque[4] = forceData.getTorque().getY();
    info.forcetorque[5] = forceData.getTorque().getZ();
    
    
    try {
      tool = (Tool)robot.findObject("tool");
    } catch (Exception e) {
      logger.info("cant find tool");
      //e.printStackTrace();
    }
    if (tool == null) {
      //no tool attached yet,return flange position
      Transformation trans = robot.getCurrentCartesianPosition(robot.getFlange()).getTransformationFromParent();
      trackingFrame f = new trackingFrame();
      f.name = robot.getFlange().getName();
      f.position[0] = trans.getX();
      f.position[1] = trans.getY();
      f.position[2] = trans.getZ();
      f.position[3] = trans.getAlphaRad();
      f.position[4] = trans.getBetaRad();
      f.position[5] = trans.getGammaRad();
      f.isMotionFrame = true;
      info.frames.add(f);
    } else {
      //find all frames link to tool and set positions to info
      for (ObjectFrame frame : tool.findFrames()) {
        Transformation trans = robot.getCurrentCartesianPosition(frame).getTransformationFromParent();
        trackingFrame f = new trackingFrame();
        f.name = frame.getName();
        f.position[0] = trans.getX();
        f.position[1] = trans.getY();
        f.position[2] = trans.getZ();
        f.position[3] = trans.getAlphaRad();
        f.position[4] = trans.getBetaRad();
        f.position[5] = trans.getGammaRad();
        if (frame.equals(tool.getDefaultMotionFrame())) {
          f.isMotionFrame = true;
        }
        info.frames.add(f);
      }
    }
    
    try {
      soc.send(GsonUtil.bean2Json(info).getBytes());
      logger.info(GsonUtil.bean2Json(info));
    } catch (IOException e) {    
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
}
