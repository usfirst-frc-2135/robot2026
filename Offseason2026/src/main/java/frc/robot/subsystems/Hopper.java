//
// Hopper Subystem - takes in Notes and delivers them to the other subsystems
//
// The hopper is composed of one motorized mechanism: a roller roller.
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
 * Hopper subsystem to control the hopper roller and rotary mechanism and provide command factories
 */
public class Hopper extends SubsystemBase
{
  // Constants
  private static final String   kSubsystemName      = "Hopper";

  private static final double   kRollerSpeedAcquire = 0.6;     // Motor direction for positive input
  private static final double   kRollerSpeedExpel   = -0.4;
  private static final double   kPulseModeDuration  = 0.5;

  // Declare device objects
  private final TalonFX         m_rollerMotorL      = new TalonFX(Ports.kCANID_HopperRollerL);
  private final TalonFX         m_rollerMotorR      = new TalonFX(Ports.kCANID_HopperRollerR);

  // Alerts
  private final Alert           m_rollerAlert       =
      new Alert(String.format("%s: Roller motor init failed!", getSubsystem( )), AlertType.kError);

  // Simulation objects
  private final TalonFXSimState m_rollerMotorLSim   = m_rollerMotorL.getSimState( );
  private final Timer           m_pulseTimer        = new Timer( );
  // Declare module variables

  // Roller variables
  private boolean               m_rollerLValid;        // Health indicator for motor 
  private boolean               m_rollerRValid;        // Health indicator for motor 
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
    m_rollerLValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorL, kSubsystemName + "Roller L",
        CTREConfigs6.hopperRollerFXConfig(false));
    m_rollerRValid = PhoenixUtil6.getInstance( ).talonFXInitialize6(m_rollerMotorR, kSubsystemName + "Roller R",
        CTREConfigs6.hopperRollerFXConfig(true));
    m_rollerAlert.set(!m_rollerLValid);

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

    // If rollermode is acquire and m_pulsemode is active
    if (m_rollerMode == HPRollerMode.ACQUIRE && m_pulseMode)
    {
      // If timer is less than 0.5 then acquire else do expel to alternate motor direction
      m_rollerMotorL.set((getMantissa(m_pulseTimer.get( )) < kPulseModeDuration) ? kRollerSpeedAcquire : kRollerSpeedExpel);
    }
    if (m_rollerMode == HPRollerMode.ACQUIRE && !m_pulseMode)
    {
      m_rollerMotorL.set(kRollerSpeedAcquire);
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

    // Set input motor voltage from the motor setting
    m_rollerMotorLSim.setSupplyVoltage(RobotController.getInputVoltage( ));

    // update for 20 msec loop

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_rollerMotorLSim.setRawRotorPosition((5300.0 / 60.0 / 50.0) * m_rollerMotorL.get( ));
    m_rollerMotorLSim.setRotorVelocity((5300.0 / 60.0) * m_rollerMotorL.get( ));

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
    PhoenixUtil6.getInstance( ).talonFXPrintFaults(m_rollerMotorL, kSubsystemName + "Roller");

    m_rollerMotorL.clearStickyFaults( );
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
    double output = 0.0;

    if (mode == HPRollerMode.HOLD)
    {
      DataLogManager.log(String.format("%s: Roller mode is unchanged - %s (%.3f)", getSubsystem( ), mode, m_rollerMotorL.get( )));
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
      m_rollerMotorL.set(output);
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
   *          requested mode
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
   * @return continuous command that runs hopper roller motors
   */
  public Command getRollerModeCommand(HPRollerMode mode)
  {
    return new InstantCommand(                          // Command with all phases declared
        ( ) -> setRollerMode(mode),                     // Init method
        this                                            // Subsytem required
    );
  }

}
