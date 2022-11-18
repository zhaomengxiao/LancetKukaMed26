package backgroundTask;

import javax.inject.Inject;

import java.io.IOException;
import java.net.SocketException;
import java.util.concurrent.TimeUnit;

import application.GsonUtil;
import com.kuka.device.common.JointPosition;
import com.kuka.geometry.Frame;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
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
		
		// your task execution starts here
		Frame f1 = robot.getCurrentCartesianPosition(robot.getFlange());
		JointPosition pt = robot.getCurrentJointPosition();
		kukaInfomation info = new kukaInfomation(); 
		
		info.setJoint1(pt.get(0));
		info.setJoint2(pt.get(1));
		info.setJoint3(pt.get(2));
		info.setJoint4(pt.get(3));
		info.setJoint5(pt.get(4));
		info.setJoint6(pt.get(5));
		info.setJoint7(pt.get(6));
		
		info.setFlange1(f1.getTransformationFromParent().getX());
		info.setFlange2(f1.getTransformationFromParent().getY());
		info.setFlange3(f1.getTransformationFromParent().getZ());
		info.setFlange4(f1.getTransformationFromParent().getAlphaRad());
		info.setFlange5(f1.getTransformationFromParent().getBetaRad());
		info.setFlange6(f1.getTransformationFromParent().getGammaRad());
				
		try {
			soc.send(GsonUtil.bean2Json(info).getBytes());
		} 
		catch (IOException e) {
			
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}