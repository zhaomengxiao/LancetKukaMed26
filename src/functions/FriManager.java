package functions;

import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRISession;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.World;
import com.kuka.math.geometry.Transformation;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.sensitivity.LBR;
import com.kuka.task.ITaskLogger;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class FriManager {
  @Inject public FriManager() {}
  
  private String m_clientName;
  private FRISession m_friSession; 
  private ObjectFrame m_friFrame;
  private FRIConfiguration m_friConfiguration;
  @Inject private ITaskLogger logger; 
  @Inject private LBRMed robot;
  @Inject private World world;

  public void setClientName(String ipAdress) {
    m_clientName = ipAdress;
  }
  
  public void initialize(){
    // Configure and start FRI session
    m_friConfiguration = FRIConfiguration.createRemoteConfiguration(robot, m_clientName);

    // Send period from LBR to client
    m_friConfiguration.setSendPeriodMilliSec(10);

    // Send period multiply with integer gives the receive period from client to robot controller. 
    // In this example it is 30 milliseconds.
    m_friConfiguration.setReceiveMultiplier(3);
    
    m_friFrame = world.getRootFrame().findFrame("/FriDynamicFrame");
    if (m_friFrame == null)
    {
      m_friFrame = world.createFrame("FriDynamicFrame", Transformation.IDENTITY);
    }
    // Select the frame from the scene graph whose transformation is changed by the client application.
    m_friConfiguration.registerTransformationProvider("FriDynamicFrame", m_friFrame);

    logger.info("Creating FRI connection to " + m_friConfiguration.getHostName());
    logger.info("SendPeriod: " + m_friConfiguration.getSendPeriodMilliSec() + "ms |"
            + " ReceiveMultiplier: " + m_friConfiguration.getReceiveMultiplier());
  }
  
  public void startFriSession() {
    m_friSession = new FRISession(m_friConfiguration);

    try
    {
      m_friSession.await(10, TimeUnit.SECONDS);
    }
    catch (TimeoutException e)
    {
      logger.error(e.getLocalizedMessage());
      m_friSession.close();
      return;
    }

    logger.info("FRI connection established.");
  }
  
  public ObjectFrame GetFriDynamicFrame() {
    return m_friFrame;
  }
  
  public void close() {
    logger.info("Close fri session to client");

    if (null != m_friSession) {
      m_friSession.close();
    }
    world.removeFrame(m_friFrame);
  }
}
