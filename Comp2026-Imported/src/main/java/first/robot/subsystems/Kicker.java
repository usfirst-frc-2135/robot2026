//
// Kicker Subsystem - takes in Notes and delivers them to the other subsystems
//
// The kicker is composed of one motorized mechanism: a roller.
//
package first.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import first.robot.Constants;
import first.robot.Constants.Ports;
import first.robot.Constants.KKConsts.KKRollerMode;
import first.robot.lib.phoenix.CTREConfigs;
import first.robot.lib.phoenix.PhoenixUtil;

import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.RobotController;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

/****************************************************************************
 * 
 * Kicker subsystem to control the kicker roller mechanism and provide command factories
 */
public class Kicker extends SubsystemBase
{
  // Constants
  private static final String   kSubsystemName      = "Kicker";

  private static final double   kRollerSpeedAcquire = 0.8;     // Motor direction for positive input
  private static final double   kRollerSpeedExpel   = -0.4;

  // Declare device objects
  private final TalonFX         m_rollerMotor       = new TalonFX(Ports.kCANID_KickerRoller, new CANBus(CANPort.CAN_S0));

  // Alerts
  private final Alert           m_rollerAlert       =
      new Alert("kickerRoller", String.format("%s: Roller motor init failed!", getName( )), Level.HIGH);

  // Simulation objects
  private final TalonFXSimState m_rollerMotorSim    = m_rollerMotor.getSimState( );

  // Declare module variables

  // Roller variables
  private boolean               m_rollerValid;        // Health indicator for motor 

  // Network tables publisher objects

  /****************************************************************************
   * 
   * Constructor
   */
  public Kicker( )
  {
    setName(kSubsystemName);

    // Roller motor init
    m_rollerValid = PhoenixUtil.getInstance( ).talonFXInitialize(m_rollerMotor, kSubsystemName + "Roller",
        CTREConfigs.kickerRollerFXConfig( ));

    m_rollerAlert.set(!m_rollerValid);

    initDashboard( );
    initialize( );
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec)
   */
  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run

    // Update network table publishers
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
    // NOTE: Kicker roller simulation is intentionally lightweight.
    // This mechanism is a lightly loaded, simple roller, so we only mirror
    // commanded output to simulated sensor state for software bring-up.

    if (!m_rollerValid)
      return;

    // Set input motor voltage from the motor setting
    m_rollerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // No detailed plant update here by design.

    // Finally, set simulated sensor readings for the roller motor.
    m_rollerMotorSim.setRawRotorPosition(
        (Constants.kKrakenX60 / Constants.kSecsPerMin / Constants.kSampleFreq) * m_rollerMotor.getThrottle( ));
    m_rollerMotorSim.setRotorVelocity((Constants.kKrakenX60 / Constants.kSecsPerMin) * m_rollerMotor.getThrottle( ));

    // Battery loading from this roller is intentionally omitted in sim.

  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {

  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setRollerMode(KKRollerMode.STOP);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (!m_rollerValid)
    {
      DataLogManager.log(String.format("%s: printFaults() skipped; roller is invalid", getName( )));
      return;
    }

    PhoenixUtil.getInstance( ).talonFXPrintFaults(m_rollerMotor, kSubsystemName + "Roller");

    m_rollerMotor.clearStickyFaults( );
  }

  /***************************************************************************
   * PRIVATE HELPERS
   ***************************************************************************/

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setRollerMode(KKRollerMode mode)
  {
    if (!m_rollerValid)
    {
      DataLogManager.log(String.format("%s: setRollerMode(%s) skipped; roller is invalid", getName( ), mode));
      return;
    }

    double output = 0.0;

    if (mode == KKRollerMode.UNCHANGED)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getName( ), mode, m_rollerMotor.getThrottle( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getName( ), mode));
        case STOP :
          output = 0.0;
          break;
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
      }

      DataLogManager.log(String.format("%s: Roller mode is now - %s", getName( ), mode));
      m_rollerMotor.setThrottle(output);
    }
  }

  /***************************************************************************
   * PUBLIC HELPERS
   ***************************************************************************/

  /***************************************************************************
   * COMMAND FACTORIES
   ***************************************************************************/

  /****************************************************************************
   * 
   * Create kicker roller mode command
   * 
   * @param mode
   *          roller mode to apply
   * @return instant command that sets kicker roller mode
   */
  public Command getRollerModeCommand(KKRollerMode mode)
  {
    return new InstantCommand(                          // Command with all phases declared
        ( ) -> setRollerMode(mode),                     // Init method
        this                                            // Subsystem required
    );
  }

}
