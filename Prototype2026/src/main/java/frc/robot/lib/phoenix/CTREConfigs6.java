
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/****************************************************************************
 * 
 * CTRE configuration structure for v6 devices
 */
public final class CTREConfigs6
{
  /****************************************************************************
   * 
   * Shooter motors - Falcon 500 (2)
   */
  public static TalonFXConfiguration shooterFXConfig( )
  {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRotaryConfig.ClosedLoopGeneral.*
    // inRotaryConfig.ClosedLoopRamps.*                           // Seconds to ramp

    shooterConfig.CurrentLimits.SupplyCurrentLimit = 80.0;        // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;   // Amps
    shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;   // Seconds
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.CurrentLimits.StatorCurrentLimit = 400.0;       // Amps
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // shooterConfig.Feedback.*
    // shooterConfig.HardwareLimitSwitch.*
    // shooterConfig.MotionMagic.*

    // Motor output settings
    // shooterConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // shooterConfig.OpenLoopRamps.*                                // Seconds to ramp

    // Slot settings
    // shooterConfig.Slot0.GravityType = *;                         // Feedforward: Mechanism is an elevator or arm
    shooterConfig.Slot0.kS = 0.0;                                   // Feedforward: Voltage or duty cylce to overcome static friction
    shooterConfig.Slot0.kG = 0.0;                                   // Feedforward: Voltage or duty cylce to overcome gravity (arbitrary feedforward)
    shooterConfig.Slot0.kV = 0.1140;                                // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    shooterConfig.Slot0.kP = 0.30;                                  // Voltage or duty cycle per velocity unit (velocity modes)
    shooterConfig.Slot0.kI = 0.0;                                   // Voltage or duty cycle per accumulated unit
    shooterConfig.Slot0.kD = 0.0;                                   // Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // shooterConfig.SoftwareLimitSwitch.*

    return shooterConfig;
  }

  public static TalonFXConfiguration upperRollerFXConfig( )
  {
    TalonFXConfiguration upperRollerConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // clawRollerConfig.ClosedLoopGeneral.*
    // clawRollerConfig.ClosedLoopRamps.*                             // Seconds to ramp

    // Current limit settings
    upperRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;         // Amps
    upperRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;    // Amps
    upperRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;    // Seconds
    upperRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    upperRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;        // Amps
    upperRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Feedback settings
    // clawRollerConfig.Feedback.*

    // Hardware limit switches - CANrange
    // upperRollerConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = canRangeID; // Stop coral on CANrange detection
    // upperRollerConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANrange;
    // upperRollerConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    // clawRollerConfig.MotionMagic.*

    // Motor output settings
    upperRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    upperRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    upperRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // clawRollerConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // clawRollerConfig.Slot0.*                                       // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)
    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    //                                                                Wrist Upward was x.x V, Elevator Downward was x.x.
    //                                                                  kG = (0.40 + 0.25) / 2
    //                                                                  kS = (0.40 - 0.25) / 2
    // clawRollerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    // clawRollerConfig.Slot0.kS = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome static friction
    // clawRollerConfig.Slot0.kG = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    // clawRollerConfig.Slot0.kV = 0.1241;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // clawRollerConfig.Slot0.kP = 0.9;                                 // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    // clawRollerConfig.Slot0.kI = 0.0;                                 // Feedback: Voltage or duty cycle per accumulated unit
    // clawRollerConfig.Slot0.kD = 0.0;                                 // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // clawRollerConfig.SoftwareLimitSwitch.*

    return upperRollerConfig;
  }

}
