package commands;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import com.kuka.geometry.World;
import com.kuka.med.devicemodel.LBRMed;

import com.kuka.task.ITaskLogger;
import java.util.concurrent.TimeUnit;
import javax.inject.Inject;
import protocols.ProtocolResult;
import units.AbstractCommand;
import units.AbstractCommandEx;
import com.kuka.motion.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartDOF;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;
import com.kuka.geometry.Tool;
public class CartesianImpedanceControl extends AbstractCommandEx {
  @Inject private LBRMed robot;
  @Inject private ITaskLogger logger;
  @Inject private World world;
  private IMotionContainer mc = null;
  private Tool tool;
  @Override
  public String GetNameString() {
    return "CartesianImpedanceControl";
  }

  @Override
  public AbstractCommand CreateCommand() {
    return new CartesianImpedanceControl();
  }

  @Override
  public ProtocolResult Execute(Object protocol) {
//    if (mc != null) {     
//      mc.cancel();
//    }
    CartesianImpedanceControlMode cartImpCtrlMode =
        new CartesianImpedanceControlMode();
    
    cartImpCtrlMode.parametrize(CartDOF.ROT).setDamping(210);
    cartImpCtrlMode.parametrize(CartDOF.X).setStiffness(5000);
    cartImpCtrlMode.parametrize(CartDOF.Y).setStiffness(5000);
    cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(5000);
    cartImpCtrlMode.setNullSpaceStiffness(100.);
    cartImpCtrlMode.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
    
    mc = robot.getFlange().moveAsync(positionHold(cartImpCtrlMode, -1, TimeUnit.SECONDS));
    
    ProtocolResult ret = new ProtocolResult();
    ret.setOperateType("CartesianImpedanceControlMode");    
    ret.setResultMsg("CartesianImpedanceControlMode ok");
    ret.setResultCode(0);
    return ret;
  }

}
