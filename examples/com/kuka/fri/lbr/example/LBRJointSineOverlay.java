package com.kuka.fri.lbr.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRIJointOverlay;
import com.kuka.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;

/**
 * Creates a FRI Session.
 */
public class LBRJointSineOverlay extends RoboticsAPIApplication
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
        friConfiguration.setSendPeriodMilliSec(5);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(_friSession);

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
        _lbr.getFlange().move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0));

        // async move with overlay ...
        _lbr.getFlange().moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay)
                .setBlendingRel(0.1));

        // ... blending into sync move with overlay
        _lbr.getFlange().move(ptp(Math.toRadians(90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
                .setJointVelocityRel(0.2)
                .addMotionOverlay(jointOverlay));
    }
}
