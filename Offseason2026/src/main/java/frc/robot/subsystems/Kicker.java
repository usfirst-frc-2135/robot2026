//
// Kicker Subystem - takes in Notes and delivers them to the other subsystems
//
// The kicker is composed of one motorized mechanism: a roller roller.
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KKConsts.KKRollerMode;
import frc.robot.Constants.Ports;
import frc.robot.lib.phoenix.CTREConfigs6;
import frc.robot.lib.phoenix.PhoenixUtil6;

/****************************************************************************
 * 
 * Kicker subsystem to control the kicker roller and rotary mechanism and provide command factories
 */
public class Kicker extends SubsystemBase
{
  // Constants
  private static final String   kSubsystemName      = "Kicker";

  private static final double   kRollerSpeedAcquire = 0.8;     // Motor direction for positive input
  private static final double   kRollerSpeedExpel   = -0.4;

  // Declare device objects
  private final TalonFX         m_rollerMotorLw     = new TalonFX(Ports.kCANID_KickerRollerLw);
  private final TalonFX         m_rollerMotorUp     = new TalonFX(Ports.kCANID_KickerRollerUp);

  // Alerts
  private final Alert           m_rollerAlert       =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState m_rollerMotorLwSim  = m_rollerMotorLw.getSimState( );

  // Declare module variables

  // Roller variables
  private boolean               m_rollerLwValid;        // Health indicator for motor 
  private boolean               m_rollerUpValid;        // Health indicator for motor 

  // Network tables publisher objects

  /****************************************************************************
   * 
   * Constructor
   */
  public Kicker( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Roller motor init
    m_rollerLwValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorLw, kSubsystemName + "Roller Lw",
        CTREConfigs6.kickerRollerFXConfig(true));
    m_rollerUpValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorUp, kSubsystemName + "Roller Up",
        CTREConfigs6.kickerRollerFXConfig(true));
    m_rollerAlert.set(!m_rollerLwValid);

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

    // Set input motor voltage from the motor setting
    m_rollerMotorLwSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // update for 20 msec loop

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rollerMotorLwSim.setRawRotorPosition((5300.0 / 60.0 / 50.0) * m_rollerMotorLw.get( ));
    m_rollerMotorLwSim.setRotorVelocity((5300.0 / 60.0) * m_rollerMotorLw.get( ));

    // SimBattery estimates loaded battery voltages

  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    // NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    // NetworkTable table = inst.getTable("kicker");

    // Initialize network tables publishers

    // Add commands
    // SmartDashboard.putData("KickerStop", getRollerModeCommand(KKRollerMode.STOP));
    // SmartDashboard.putData("KickerAcquire", getRollerModeCommand(KKRollerMode.ACQUIRE));
    // SmartDashboard.putData("KickerExpel", getRollerModeCommand(KKRollerMode.EXPEL));
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
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotorLw, kSubsystemName + "Roller");

    m_rollerMotorLw.clearStickyFaults( );
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
  private void setRollerMode(KKRollerMode mode)
  {
    double output = 0.0;

    if (mode == KKRollerMode.HOLD)
    {
      DataLogManager
          .log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotorLw.get( )));
    }
    else
    {
      switch (mode)
      {
        default :
          DataLogManager.log(String.format("%s: Roller mode is invalid: %s", getSubsystem( ), mode));
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

      DataLogManager.log(String.format("%s: Roller mode is now - %s", getSubsystem( ), mode));
      m_rollerMotorLw.set(output);
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// PUBLIC HELPERS ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////
  ///////////////////////// COMMAND FACTORIES ////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Create kicker roller mode command
   * 
   * @param mode
   *          roller mode to apply
   * @return continuous command that runs kicker roller motors
   */
  public Command getRollerModeCommand(KKRollerMode mode)
  {
    return new InstantCommand(                          // Command with all phases declared
        ( ) -> setRollerMode(mode),                     // Init method
        this                                            // Subsytem required
    );
  }

}
