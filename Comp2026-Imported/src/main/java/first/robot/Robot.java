
package first.robot;

import java.util.Optional;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.framework.TimedRobot;
import org.wpilib.net.WebServer;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Filesystem;
import org.wpilib.system.RobotController;

import com.pathplanner.lib.commands.FollowPathCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot
{
  private static final boolean m_isComp            = detectRobot( );        // Detect whether this is the competition/practice robot
  private final RobotContainer m_robotContainer    = new RobotContainer(this); // Main robot container
  private Command              m_autonomousCommand = null;                  // The autonomous command selected by RobotContainer
  private static boolean       m_loadAutoCommand   = true;                  // Flag to indicate that the autonomous command needs to be reloaded
  private Optional<Alliance>   m_alliance          = Optional.empty( );     // Current alliance color, used to detect changes in alliance color

  /****************************************************************************
   * 
   * This function runs when the Robot class is first started and used for initialization.
   */
  public Robot( )
  {
    // Starts recording to data log
    DataLogManager.start( );
    DriverStation.startDataLog(DataLogManager.getLog( )); // Enables logging of joystick data

    // Log when commands initialize, interrupt, and end states
    CommandScheduler.getInstance( ).onCommandInitialize(cmd -> DataLogManager.log(String.format("%s: Init", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandInterrupt(cmd -> DataLogManager.log(String.format("%s: Interrupt", cmd.getName( ))));
    CommandScheduler.getInstance( ).onCommandFinish(cmd -> DataLogManager.log(String.format("%s: End", cmd.getName( ))));

    // Start the web server for remote dashboard layout (only on a real Systemcore)
    if (RobotBase.isReal( ))
    {
      try
      {
        WebServer.start(5800, Filesystem.getDeployDirectory( ).getPath( ));
      }
      catch (Exception e)
      {
        DataLogManager.log(String.format("Failed to start WebServer: %s", e.toString( )));
      }
    }

    // PathPlanner warmup command - recommended by PathPlanner docs
    CommandScheduler.getInstance( ).schedule(FollowPathCommand.warmupCommand( ).withName("PathPlanner - warmupCommand"));
  }

  /****************************************************************************
   * 
   * This function is called every 20 msec robot loop, no matter the mode. Use this for items like
   * diagnostics that you want to run during Disabled, Autonomous, Teleoperated and Utility.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic( )
  {
    // Runs the Command Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    //
    CommandScheduler.getInstance( ).run( );

    // Call the robotPeriodic() method in RobotContainer to handle any periodic tasks for subsystems
    m_robotContainer.robotPeriodic( );
  }

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit( )
  {
    datalogMatchBanner("disabledInit");

    m_robotContainer.disabledInit( );
  }

  /**
   * This function is called periodically while in Disabled mode.
   */
  @Override
  public void disabledPeriodic( )
  {
    detectAutoDashboardChange( );
  }

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit( )
  {
    datalogMatchBanner("autonInit");

    cancelAnyRunningAutonomousCommand( );

    // Handle any commands that need to be scheduled when entering Autonomous mode
    m_robotContainer.autoInit( );

    // NOTE: Intentionally schedule only a PRELOADED auto command here.
    // Do not call getAutonomousCommand() from autonomousInit(). Path parsing/loading can take
    // hundreds of milliseconds and would delay robot movement at the start of autonomous.
    // The command is prebuilt in disabledPeriodic() via detectAutoDashboardChange().
    //
    // Schedule the preloaded autonomous command selected by RobotContainer.
    if (m_autonomousCommand != null)
    {
      DataLogManager.log(String.format("autonInit: scheduling preloaded auto command: %s", m_autonomousCommand.getName( )));
      CommandScheduler.getInstance( ).schedule(m_autonomousCommand);
    }
    else
    {
      DataLogManager.log("autonInit: WARNING - m_autonomousCommand is null; no autonomous command scheduled");
    }
  }

  /**
   * This function is called periodically during Autonomous mode.
   */
  @Override
  public void autonomousPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit( )
  {
    datalogMatchBanner("teleopInit");

    // Make sure that the autonomous command stops running when Teleop starts running
    cancelAnyRunningAutonomousCommand( );

    // Handle any commands that need to be scheduled when entering Teleop mode
    m_robotContainer.teleopInit( );
  }

  /**
   * This function is called periodically during Teleop mode (operator control).
   */
  @Override
  public void teleopPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Simulation mode.
   */
  @Override
  public void simulationInit( )
  {}

  /**
   * This function is called periodically during Simulation mode.
   */
  @Override
  public void simulationPeriodic( )
  {}

  /****************************************************************************
   * 
   * This function is called once each time the robot enters Utility mode.
   */
  @Override
  public void utilityInit( )
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance( ).cancelAll( );
  }

  /**
   * This function is called periodically during Utility mode.
   */
  @Override
  public void utilityPeriodic( )
  {}

  /****************************************************************************/

  /****************************************************************************
   * 
   * Robot detection process to differentiate between competition and practice robots.
   */
  public static boolean isComp( )
  {
    return m_isComp;
  }

  private static boolean detectRobot( )
  {
    // Detect which robot/Systemcore is being used by reading the serial number. 
    // Used to differentiate between the competition and practice robots.
    String serialNum = null;
    if (RobotBase.isSimulation( ))
    {
      serialNum = null;
      DataLogManager.log("robotContainer: Simulation mode detected, using simulated serial number path");
    }
    else
    {
      try
      {
        serialNum = RobotController.getSerialNumber( );
      }
      catch (Throwable t)
      {
        serialNum = null;
        DataLogManager
            .log(String.format("robotContainer: WARNING - failed to read Systemcore serial number (%s)", t.toString( )));
      }
    }

    String robotName = "UNKNOWN";
    boolean isComp = false;

    DataLogManager.log(String.format("robotContainer: Systemcore SN: %s", serialNum));
    if (serialNum == null)
    {
      robotName = "SIMULATION";
    }
    else if (serialNum.equals(Constants.kCompSN))
    {
      isComp = true;
      robotName = "COMPETITION (A)";
    }
    else if (serialNum.equals(Constants.kPracticeSN))
    {
      isComp = false;
      robotName = "PRACTICE (B)";
    }
    DataLogManager.log(String.format("robotContainer: Detected the %s robot (Systemcore)!", robotName));

    return isComp;
  }

  /****************************************************************************
   * 
   * Display a mode change banner for the match type and number
   */
  private static void datalogMatchBanner(String msg)
  {
    final String printBarrier = "========================================================================";

    DataLogManager.log(String.format(printBarrier));
    DataLogManager.log(String.format("%s: Match %s %s, %s Alliance", msg, MatchState.getMatchType( ).toString( ),
        MatchState.getMatchNumber( ), MatchState.getAlliance( ).toString( )));
    DataLogManager.log(String.format(printBarrier));
  }

  /****************************************************************************
   * 
   * Detect dashboard/settings changes that require autonomous command reload.
   */
  private void detectAutoDashboardChange( )
  {
    if (m_robotContainer.autoDelayIsChanged( ))
    {
      m_loadAutoCommand = true;
    }

    Optional<Alliance> alliance = MatchState.getAlliance( );
    if (!m_alliance.equals(alliance))
    {
      m_loadAutoCommand = true;
      m_alliance = alliance;
    }

    if (m_loadAutoCommand)
    {
      // Intentionally preload autonomous command while disabled. This keeps autonomousInit()
      // lightweight and avoids startup lag from path file loading/parsing at mode transition.
      m_autonomousCommand = m_robotContainer.getAutonomousCommand( );
      m_loadAutoCommand = false; // Preload only once per reload request
    }
  }

  /****************************************************************************
   * 
   * Cancel any running autonomous commands
   */
  private void cancelAnyRunningAutonomousCommand( )
  {
    if (m_autonomousCommand != null && m_autonomousCommand.isScheduled( ))
    {
      m_autonomousCommand.cancel( );
    }
  }

  /****************************************************************************
   * 
   * Signal autonomous command to reload
   */
  public static void reloadAutonomousCommand(String optionName)
  {
    DataLogManager.log(String.format("Auto change! - %s", optionName));
    m_loadAutoCommand = true;
  }

}
