
//
// HID command class - Game controller feedback (XBox rumble) commands
//
package first.robot.lib;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;

/****************************************************************************
 * 
 * HID class to provide rumble command factory with timing
 */
public class HID
{
  // Constants

  // Member objects
  private String            m_name          = "";
  private CommandGenericHID m_driver;
  private CommandGenericHID m_operator;
  private Timer             m_timerDriver   = new Timer( );
  private Timer             m_timerOperator = new Timer( );
  private Time              m_driverDuration;
  private Time              m_operatorDuration;

  /****************************************************************************
   * 
   * Constructor
   * 
   * @param driver
   *          driver gamepad to initialize
   * @param operator
   *          operator gamepad to initialize
   */
  public HID(CommandGenericHID driver, CommandGenericHID operator)
  {
    setName("HID");

    m_driver = driver;
    m_operator = operator;

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

    if (m_timerDriver.isRunning( ) && m_timerDriver.hasElapsed(m_driverDuration.in(Seconds)))
    {
      m_timerDriver.stop( );
      setHIDRumbleDriver(false, Seconds.of(0), 0);
    }

    if (m_timerOperator.isRunning( ) && m_timerOperator.hasElapsed(m_operatorDuration.in(Seconds)))
    {
      m_timerOperator.stop( );
      setHIDRumbleOperator(false, Seconds.of(0), 0);
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

  /***************************************************************************
   * PRIVATE HELPERS
   ***************************************************************************/

  /****************************************************************************
   * 
   * Set HID driver rumble based on the requested on/off and intensity
   * 
   * @param rumbleOn
   *          request rumble on or off
   * @param duration
   *          request a time for the rumble to last
   * @param intensity
   *          requested rumble strength
   */
  private void setHIDRumbleDriver(boolean rumbleOn, Time duration, double intensity)
  {
    DataLogManager
        .log(String.format("%s: Rumble driver: %s duration %s intensity: %.1f", getName( ), rumbleOn, duration, intensity));

    if (rumbleOn)
    {
      m_timerDriver.restart( );
      m_driverDuration = duration;
    }

    m_driver.setRumble(RumbleType.LEFT_RUMBLE, (rumbleOn) ? intensity : 0.0);
    m_driver.setRumble(RumbleType.RIGHT_RUMBLE, (rumbleOn) ? intensity : 0.0);
  }

  /****************************************************************************
   * 
   * Set HID operator rumble based on the requested on/off and intensity
   * 
   * @param rumbleOn
   *          request rumble on or off
   * @param duration
   *          request a time for the rumble to last
   * @param intensity
   *          requested rumble strength
   */
  private void setHIDRumbleOperator(boolean rumbleOn, Time duration, double intensity)
  {
    DataLogManager
        .log(String.format("%s Rumble operator: %s duration %s intensity: %.1f", getName( ), rumbleOn, duration, intensity));

    if (rumbleOn)
    {
      m_timerOperator.restart( );
      m_operatorDuration = duration;
    }

    m_operator.setRumble(RumbleType.LEFT_RUMBLE, (rumbleOn) ? intensity : 0.0);
    m_operator.setRumble(RumbleType.RIGHT_RUMBLE, (rumbleOn) ? intensity : 0.0);
  }

  /***************************************************************************
   * COMMAND FACTORIES
   ***************************************************************************/

  /****************************************************************************
   * 
   * Create HID set rumble for driver controller command
   * 
   * @param driverRumble
   *          rumble the driver gamepad
   * @param duration
   *          request a time for the rumble to last
   * @param intensity
   *          stength of the rumble [0.0 .. 1.0]
   * @return instant command that rumbles the gamepad
   */
  public Command getHIDRumbleDriverCommand(boolean driverRumble, Time duration, double intensity)
  {

    return new InstantCommand(                                        // Command that runs exactly once
        ( ) -> setHIDRumbleDriver(driverRumble, duration, intensity)  // Method to call
    ).withName("HIDRumbleDriver").ignoringDisable(true);
  }

  /****************************************************************************
   * 
   * Create HID set rumble for operator controller command
   * 
   * @param operatorRumble
   *          rumble the operator gamepad
   * @param duration
   *          request a time for the rumble to last
   * @param intensity
   *          stength of the rumble [0.0 .. 1.0]
   * @return instant command that rumbles the gamepad
   */
  public Command getHIDRumbleOperatorCommand(boolean operatorRumble, Time duration, double intensity)
  {

    return new InstantCommand(                                            // Command that runs exactly once
        ( ) -> setHIDRumbleOperator(operatorRumble, duration, intensity)  // Method to call
    ).withName("HIDRumbleOperator").ignoringDisable(true);
  }
}
