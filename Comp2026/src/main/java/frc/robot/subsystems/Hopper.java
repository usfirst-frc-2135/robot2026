//
// Hopper Subsystem - takes in Notes and delivers them to the other subsystems
//
// The hopper is composed of one motorized mechanism: a roller.
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HPConsts.HPRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Hopper subsystem to control the hopper roller mechanism and provide command factories
 */
public class Hopper extends SubsystemBase
{
  // Constants
  private static final String   kSubsystemName      = "Hopper";

  private static final double   kRollerSpeedAcquire = 0.6;     // Motor direction for positive input
  private static final double   kRollerSpeedExpel   = -0.4;
  private static final double   kPulseModeDuration  = 0.5;

  // Declare device objects
  private final TalonFX         m_rollerMotor       = new TalonFX(Ports.kCANID_HopperRoller);

  // Alerts
  private final Alert           m_rollerAlert       =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState m_rollerMotorSim    = m_rollerMotor.getSimState( );
  private final Timer           m_pulseTimer        = new Timer( );
  // Declare module variables

  // Roller variables
  private boolean               m_rollerValid;        // Health indicator for motor 
  private HPRollerMode          m_rollerMode        = HPRollerMode.STOP;
  private boolean               m_pulseMode         = false;

  // Network tables publisher objects

  /****************************************************************************
   * 
   * Constructor
   */
  public Hopper( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Roller motor init
    m_rollerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotor, kSubsystemName + "Roller",
        CTREConfigs6.hopperRollerFXConfig( ));

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

    if (!m_rollerValid)
      return;

    // If roller mode is ACQUIRE and pulse mode is active
    if (m_rollerMode == HPRollerMode.ACQUIRE && m_pulseMode)
    {
      // For the first half-second acquire, then reverse to expel for pulsing behavior
      m_rollerMotor.set((getMantissa(m_pulseTimer.get( )) < kPulseModeDuration) ? kRollerSpeedAcquire : kRollerSpeedExpel);
    }
    else if (m_rollerMode == HPRollerMode.ACQUIRE && !m_pulseMode)
    {
      m_rollerMotor.set(kRollerSpeedAcquire);
    }
  }

  /****************************************************************************
   * 
   * Periodic actions that run every scheduler loop time (20 msec) during simulation
   */
  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run during simulation
    // NOTE: Hopper roller simulation is intentionally lightweight.
    // This is a simple, lightly loaded roller, so we mirror commanded output
    // into simulated sensor state without a detailed mechanical plant model.

    // Set input motor voltage from the motor setting
    m_rollerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // No detailed plant update here by design.

    // Finally, set simulated sensor readings for the roller motor
    m_rollerMotorSim.setRawRotorPosition((5300.0 / 60.0 / 50.0) * m_rollerMotor.get( ));
    m_rollerMotorSim.setRotorVelocity((5300.0 / 60.0) * m_rollerMotor.get( ));

    // Battery loading from this roller is intentionally omitted in sim.

  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    // NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    // NetworkTable table = inst.getTable("hopper");

    // Initialize network tables publishers

    // Add commands
    // SmartDashboard.putData("HopperStop", getRollerModeCommand(HPRollerMode.STOP));
    // SmartDashboard.putData("HopperAcquire", getRollerModeCommand(HPRollerMode.ACQUIRE));
    // SmartDashboard.putData("HopperExpel", getRollerModeCommand(HPRollerMode.EXPEL));
  }

  // Put methods for controlling this subsystem here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize subsystem during robot mode changes
   */
  public void initialize( )
  {
    setRollerMode(HPRollerMode.STOP);
  }

  /****************************************************************************
   * 
   * Write out hardware faults and reset sticky faults
   */
  public void printFaults( )
  {
    if (!m_rollerValid)
    {
      DataLogManager.log(String.format("%s: printFaults() skipped; roller is invalid", getSubsystem( )));
      return;
    }

    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotor, kSubsystemName + "Roller");

    m_rollerMotor.clearStickyFaults( );
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PRIVATE HELPERS //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Set roller speed based on requested mode
   * 
   * @param mode
   *          requested speed
   */
  private void setRollerMode(HPRollerMode mode)
  {
    if (!m_rollerValid)
    {
      m_rollerMode = HPRollerMode.STOP;
      return;
    }

    double output = 0.0;

    if (mode == HPRollerMode.UNCHANGED)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotor.get( )));
    }
    else
    {
      m_pulseTimer.restart( );
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getSubsystem( ), mode));
        case STOP :
          m_pulseMode = false;
          output = 0.0;

          break;
        case ACQUIRE :
          output = kRollerSpeedAcquire;
          break;
        case EXPEL :
          output = kRollerSpeedExpel;
          break;
      }

      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_rollerMotor.set(output);
    }
    m_rollerMode = mode;
  }

  /****************************************************************************
   * 
   * Return the mantissa of a floating point (double) value
   * 
   * @param num
   *          number to return mantissa for
   * @return decimal mantissa with integer remove
   */
  private double getMantissa(double num)
  {
    double numInt = Math.floor(num);
    return num - numInt;
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Update the pulse mode value for external control of motor pulsing
   * 
   * @param mode
   *          requested pulse-mode state
   */
  public void setPulseMode(boolean mode)
  {
    m_pulseMode = mode;
    DataLogManager.log(String.format("%s: Set PULSE MODE: %s", getSubsystem( ), mode));
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create hopper roller mode command
   * 
   * @param mode
   *          roller mode to apply
   * @return instant command that sets hopper roller mode
   */
  public Command getRollerModeCommand(HPRollerMode mode)
  {
    return new InstantCommand(                          // Command with all phases declared
        ( ) -> setRollerMode(mode),                     // Init method
        this                                            // Subsystem required
    );
  }

}
