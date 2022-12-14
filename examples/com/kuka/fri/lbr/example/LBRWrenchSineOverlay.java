package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRIJointOverlay;
import com.kuka.fri.FRISession;
import com.kuka.fri.common.ClientCommandMode;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartDOF;
import com.kuka.sensitivity.LBR;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;

/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * wrenches can be additionally commanded via FRI.
 */
public class LBRWrenchSineOverlay extends RoboticsAPIApplication
{
    private String _clientName;
    private FRISession _friSession;

    @Inject
    private LBR _lbr;

    @Override
    public void initialize()
    {
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "127.0.0.1";
    }

    @Override
    public void dispose()
    {
        getLogger().info("Close connection to client");

        if (null != _friSession)
        {
            _friSession.close();
        }
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for wrench mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = new FRISession(friConfiguration);
        FRIJointOverlay wrenchOverlay = new FRIJointOverlay(_friSession, ClientCommandMode.WRENCH);

        // wait until FRI session is ready to switch to command mode
        try
        {
            _friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // move to start pose
        _lbr.getFlange().move(ptp(Math.toRadians(90), Math.toRadians(-60), .0, Math.toRadians(60), .0, Math.toRadians(-60), .0));

        // start PositionHold with overlay
        CartesianImpedanceControlMode ctrMode = new CartesianImpedanceControlMode();
        ctrMode.parametrize(CartDOF.TRANSL).setStiffness(100);
        PositionHold posHold = new PositionHold(ctrMode, 20, TimeUnit.SECONDS);

        _lbr.getFlange().move(posHold.addMotionOverlay(wrenchOverlay));
    }
}
