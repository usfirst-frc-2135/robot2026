//
// Kicker Subystem - takes in Notes and delivers them to the other subsystems
//
// The kicker is composed of one motorized mechanism: a roller roller.
//
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private static final double   kRollerSpeedAcquire = 0.5;     // Motor direction for positive input
  private static final double   kRollerSpeedExpel   = -0.4;

  // Declare device objects
  private final TalonFX         m_rollerMotor       = new TalonFX(Ports.kCANID_KickerRoller);

  // Alerts
  private final Alert           m_rollerAlert       =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState m_rollerMotorSim    = m_rollerMotor.getSimState( );

  // Declare module variables

  // Roller variables
  private boolean               m_rollerValid;        // Health indicator for motor 

  // Network tables publisher objects
  private DoublePublisher       m_rollSpeedPub;
  private DoublePublisher       m_rollSupCurPub;

  /****************************************************************************
   * 
   * Constructor
   */
  public Kicker( )
  {
    setName(kSubsystemName);
    setSubsystem(kSubsystemName);

    // Roller motor init
    m_rollerValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotor, kSubsystemName + "Roller",
        CTREConfigs6.kickerRollerFXConfig( ));

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
    m_rollSpeedPub.set(m_rollerMotor.get( ));
    m_rollSupCurPub.set(m_rollerMotor.get( ));
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
    m_rollerMotorSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // update for 20 msec loop

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rollerMotorSim.setRawRotorPosition((5300 / 60 / 50) * m_rollerMotor.get( ));
    m_rollerMotorSim.setRotorVelocity((5300 / 60) * m_rollerMotor.get( ));

    // SimBattery estimates loaded battery voltages

  }

  /****************************************************************************
   * 
   * Initialize dashboard widgets
   */
  private void initDashboard( )
  {
    // Get the default instance of NetworkTables that was created automatically when the robot program starts
    NetworkTableInstance inst = NetworkTableInstance.getDefault( );
    NetworkTable table = inst.getTable("kicker");

    // Initialize network tables publishers
    m_rollSpeedPub = table.getDoubleTopic("rollSpeed").publish( );
    m_rollSupCurPub = table.getDoubleTopic("rollSupCur").publish( );

    // Add commands
    SmartDashboard.putData("KickerStop", getRollerModeCommand(KKRollerMode.STOP));
    SmartDashboard.putData("KickerAcquire", getRollerModeCommand(KKRollerMode.ACQUIRE));
    SmartDashboard.putData("KickerExpel", getRollerModeCommand(KKRollerMode.EXPEL));
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
  private void setRollerMode(KKRollerMode mode)
  {
    double output = 0.0;

    if (mode == KKRollerMode.HOLD)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotor.get( )));
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
      m_rollerMotor.set(output);
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
