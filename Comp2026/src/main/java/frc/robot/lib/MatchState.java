
//
// Match State class - Manage match timing and state
//
package frc.robot.lib;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/****************************************************************************
 * 
 * HID class to provide rumble command factory
 */
public class MatchState
{
  // Constants

  // Member objects
  private String m_name = new String( );

  /****************************************************************************
   * 
   * Constructor
   * 
   * @param driver
   *          driver gamepad to initialize
   * @param operator
   *          operator gamepad to initialize
   */
  public MatchState( )
  {
    setName("MatchState");

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Getter and setter for managing the class name
   */
  private void setName(String name)
  {
    m_name = name;
  }

  public String getName( )
  {
    return m_name;
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  public void periodic( )
  {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime( ));

    // Manage robot state based on match time
    if (DriverStation.isAutonomous( ) || DriverStation.isTeleop( ))
    {
      SmartDashboard.putNumber("ShiftTime", timeLeftInShiftSeconds(DriverStation.getMatchTime( )));
    }
    else
    {
      SmartDashboard.putNumber("ShiftTime", 0.0);
    }
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Initialize dashboard widgets

    SmartDashboard.putNumber("ShiftTime", 0.0);
  }

  // Put methods for controlling this class here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize class during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getName( )));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Return if the robot is on the requested alliance
   * 
   * @return true if robot is on Blue alliance
   */
  public static boolean isBlue( )
  {
    return DriverStation.getAlliance( ).orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue);
  }

  /**
   * @return true if robot is on Red alliance
   */
  public static boolean isRed( )
  {
    return !isBlue( );
  }

  /****************************************************************************
   * 
   * Return if Blue alliance won the auto period
   * 
   * @return true if Blue alliance won auto
   */
  public static boolean blueWonAuto( )
  {
    String matchInfo = DriverStation.getGameSpecificMessage( );
    if (matchInfo != null && matchInfo.length( ) > 0)
    {
      return matchInfo.charAt(0) == 'B';
    }
    // Safe default if data isn't ready yet
    return false;
  }

  /****************************************************************************
   * 
   * Return the time left in each shift
   * 
   * @param currentMatchTime
   *          current Match time in seconds (countdown)
   * @return time remaining in the current shift
   */
  public static int timeLeftInShiftSeconds(double currentMatchTime)
  {
    if (currentMatchTime >= 140)
    {
      return (int) (currentMatchTime - 140);  // Autonomous
    }
    else if (currentMatchTime >= 130 && currentMatchTime <= 140)
    {
      return (int) (currentMatchTime - 130);  // Transition
    }
    else if (currentMatchTime >= 105 && currentMatchTime <= 130)
    {
      return (int) (currentMatchTime - 105);  // First odd shift
    }
    else if (currentMatchTime >= 80 && currentMatchTime <= 105)
    {
      return (int) (currentMatchTime - 80);   // First even shift
    }
    else if (currentMatchTime >= 55 && currentMatchTime <= 80)
    {
      return (int) (currentMatchTime - 55);   // Last odd shift
    }
    else if (currentMatchTime >= 30 && currentMatchTime <= 55)
    {
      return (int) (currentMatchTime - 30);   // Last even shift
    }
    else
    {
      return (int) currentMatchTime;          // End game
    }
  }

  /****************************************************************************
   * 
   * Return if the shift time is for Blue alliance
   * 
   * @param currentMatchTime
   *          current Match time in seconds (countdown)
   * @return true of an active Blue alliance shift
   */
  public static boolean isCurrentShiftBlue(double currentMatchTime)
  {
    if (currentMatchTime >= 105 && currentMatchTime <= 130)
    {
      return blueWonAuto( ) ? false : true;
    }
    else if (currentMatchTime >= 80 && currentMatchTime <= 105)
    {
      return blueWonAuto( ) ? true : false;
    }
    else if (currentMatchTime >= 55 && currentMatchTime <= 80)
    {
      return blueWonAuto( ) ? false : true;
    }
    else if (currentMatchTime >= 30 && currentMatchTime <= 55)
    {
      return blueWonAuto( ) ? true : false;
    }
    else
    {
      return true;
    }
  }

  /****************************************************************************
   * 
   * Create HID set rumble for operator controller command
   * 
   * @return instant command that rumbles the gamepad
   */
  public static boolean currentShiftIsYours( )
  {
    double currentMatchTime = DriverStation.getMatchTime( );
    boolean isBlueShift = isCurrentShiftBlue(currentMatchTime);
    if (isBlue( ))
    {
      return isBlueShift;
    }
    else
    {
      return !isBlueShift;
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

}
