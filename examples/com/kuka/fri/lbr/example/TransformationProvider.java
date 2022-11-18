package com.kuka.fri.lbr.example;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRISession;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Transformation;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.sensitivity.LBR;
import com.kuka.threading.ThreadUtil;

/**
 * Creates a FRI Session.
 */
public class TransformationProvider extends RoboticsAPIApplication
{
    private String _clientName;
    private FRISession _friSession;

    private ObjectFrame _objectBase;
    private ObjectFrame _objectTip;

    @Inject
    private LBR _lbr;

    @Inject
    private World _world;

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

        _world.removeFrame(_objectBase);
    }

    @Override
    public void run()
    {
        // Create some frames
        _objectBase = _world.getRootFrame().findFrame("/FRIexampleBase");
        if (_objectBase == null)
        {
            _objectBase = _world.createFrame("FRIexampleBase", Transformation.IDENTITY);
        }

        _objectTip = _world.getRootFrame().findFrame("/FRIexampleBase/FRIexampleTip");
        if (_objectTip == null)
        {
            _objectTip = _world.createFrame("FRIexampleTip", _objectBase, Transformation.ofDeg(10.0, 10.0, 10.0, 45.0, 45.0, 45.0));
        }

        // Configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);

        // Send period from LBR to client
        friConfiguration.setSendPeriodMilliSec(10);

        // Send period multiply with integer gives the receive period from client to robot controller. 
        // In this example it is 30 milliseconds.
        friConfiguration.setReceiveMultiplier(3);

        // Select the frame from the scene graph whose transformation is changed by the client application.
        friConfiguration.registerTransformationProvider("PBase", _objectBase);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = new FRISession(friConfiguration);

        try
        {
            _friSession.await(10, TimeUnit.SECONDS);
        }
        catch (TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // Output
        getLogger().info("Transformation from World of");
        for (int i = 0; i < 100; i++)
        {
            ThreadUtil.milliSleep(15);
            getLogger().info("Frame objectBase:\n" + _objectBase.calculateTransformationFromTreeRoot());
            getLogger().info("Frame objectTip:\n" + _objectTip.calculateTransformationFromTreeRoot());
        }
    }
}
