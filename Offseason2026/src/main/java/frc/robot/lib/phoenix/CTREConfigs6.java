
// Phoenix 6 configurations

package frc.robot.lib.phoenix;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

/****************************************************************************
 * 
 * CTRE configuration structure for v6 devices
 */
public final class CTREConfigs6
{

  /****************************************************************************
   * 
   * Intake roller motor - Kraken X60
   * 
   * @return inRollerConfig
   */
  public static TalonFXConfiguration intakeRollerFXConfig(boolean inverted)
  {
    TalonFXConfiguration inRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    inRollerConfig.CurrentLimits.SupplyCurrentLimit = 20.0;       // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0;  // Amps
    inRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    inRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    inRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    inRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;   // Percentage
    inRollerConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    inRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return inRollerConfig;
  }

  /****************************************************************************
   * 
   * Intake rotary motor - Kraken X44
   * 
   * @param min
   *          minimum angle of rotation
   * @param max
   *          maximum angle of rotation (must be greater than min)
   * @param ccPort
   *          CANcoder port number
   * @param gearRatio
   *          gear box ratio
   * @return inRackConfig
   */
  public static TalonFXConfiguration intakeRackFXConfig(double gearRatio)
  {
    TalonFXConfiguration inRackConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRackConfig.ClosedLoopGeneral.*
    // inRackConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    inRackConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    inRackConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    inRackConfig.CurrentLimits.SupplyCurrentLowerTime = 0.200;  // Seconds
    inRackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    inRackConfig.CurrentLimits.StatorCurrentLimit = 400.0;      // Amps
    inRackConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // Feedback settings
    // inRackConfig.Feedback.FeedbackRemoteSensorID
    // inRackConfig.Feedback.FeedbackSensorSource
    inRackConfig.Feedback.SensorToMechanismRatio = 1.0;
    inRackConfig.Feedback.RotorToSensorRatio = 1.0;

    // Hardware limit switches - NONE
    // inRackConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    inRackConfig.MotionMagic.MotionMagicCruiseVelocity = 60.0;      // Rotations / second
    inRackConfig.MotionMagic.MotionMagicAcceleration = 200.0;       // Rotations / second ^ 2
    inRackConfig.MotionMagic.MotionMagicJerk = 2000.0;              // Rotations / second ^ 3

    // Motor output settings
    inRackConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    inRackConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    inRackConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // inRackConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // kG = (0.40 + 0.25) / 2
    // kS = (0.40 - 0.25) / 2
    inRackConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    inRackConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    inRackConfig.Slot0.kS = 0.07;                                 // Feedforward: Voltage or duty cycle to overcome static friction
    inRackConfig.Slot0.kG = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    inRackConfig.Slot0.kV = 0.1129;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // NOTE: Motion Magic settings are scaled by gear ration when using a FusecCANCoder
    inRackConfig.Slot0.kP = 4.0;                                  // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    inRackConfig.Slot0.kI = 0.0;                                  // Feedback: Voltage or duty cycle per accumulated unit
    inRackConfig.Slot0.kD = 0.05;                                  // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // inRackConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    // inRackConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // inRackConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    // inRackConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return inRackConfig;
  }

  /****************************************************************************
   * 
   * Hopper roller motor - Kraken X60
   * 
   * @return hpRollerConfig
   */
  public static TalonFXConfiguration hopperRollerFXConfig(boolean inverted)
  {
    TalonFXConfiguration hpRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    hpRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    hpRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    hpRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hpRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    hpRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    hpRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;  // Percentage
    hpRollerConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    hpRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    hpRollerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.100; // Seconds to ramp

    return hpRollerConfig;
  }

  /****************************************************************************
   * 
   * Kicker roller motor - Kraken X60
   * 
   * @return kkRollerConfig
   */
  public static TalonFXConfiguration kickerRollerFXConfig(boolean inverted)
  {
    TalonFXConfiguration kkRollerConfig = new TalonFXConfiguration( );

    // Current limit settings
    kkRollerConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    kkRollerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.100;  // Seconds
    kkRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kkRollerConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    kkRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motor output settings
    kkRollerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    kkRollerConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    kkRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return kkRollerConfig;
  }

  /****************************************************************************
   * 
   * Launcher motors - Kraken X60 (2)
   * 
   * @param inverted
   * 
   * @return launcherConfig
   */
  public static TalonFXConfiguration launcherFXConfig(boolean inverted)
  {
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // inRackConfig.ClosedLoopGeneral.*
    // inRackConfig.ClosedLoopRamps.*                             // Seconds to ramp

    launcherConfig.CurrentLimits.SupplyCurrentLimit = 35.0;          // Amps
    launcherConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;     // Amps
    launcherConfig.CurrentLimits.SupplyCurrentLowerTime = 0.001;     // Seconds
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // launcherConfig.CurrentLimits.StatorCurrentLimit = 200.0;      // Amps
    // launcherConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // launcherConfig.Feedback.*
    // launcherConfig.HardwareLimitSwitch.*
    // launcherConfig.MotionMagic.*

    // Motor output settings
    // launcherConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;
    launcherConfig.MotorOutput.Inverted = (inverted) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Open Loop settings
    // launcherConfig.OpenLoopRamps.*                                  // Seconds to ramp

    // Slot settings
    // launcherConfig.Slot0.GravityType = *; 
    launcherConfig.Slot0.kS = 0.3;                                     // Feedforward: Voltage or duty cycle to overcome static friction
    launcherConfig.Slot0.kG = 0.0;                                     // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    launcherConfig.Slot0.kV = 0.1193;                                  // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    launcherConfig.Slot0.kP = 0.465;                                    // Voltage or duty cycle per velocity unit (velocity modes)
    launcherConfig.Slot0.kI = 0.0;                                     // Voltage or duty cycle per accumulated unit
    launcherConfig.Slot0.kD = 0.0;                                     // Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // launcherConfig.SoftwareLimitSwitch.*

    return launcherConfig;
  }

  /****************************************************************************
   * 
   * Launcher hood motor - Kraken X44
   * 
   * @param gearRatio
   *          gear box ratio
   * @return launcherHoodConfig
   */
  public static TalonFXConfiguration hoodFXConfig(double gearRatio)
  {
    TalonFXConfiguration launcherHoodConfig = new TalonFXConfiguration( );

    // Closed Loop settings
    // launcherHoodConfig.ClosedLoopGeneral.*
    // launcherHoodConfig.ClosedLoopRamps.*                           // Seconds to ramp

    // Current limit settings
    launcherHoodConfig.CurrentLimits.SupplyCurrentLimit = 25.0;       // Amps
    launcherHoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 25.0;  // Amps
    launcherHoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.200;  // Seconds
    launcherHoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherHoodConfig.CurrentLimits.StatorCurrentLimit = 400.0;      // Amps
    launcherHoodConfig.CurrentLimits.StatorCurrentLimitEnable = false;

    // Feedback settings
    // launcherHoodConfig.Feedback.FeedbackRemoteSensorID
    // launcherHoodConfig.Feedback.FeedbackSensorSource
    launcherHoodConfig.Feedback.SensorToMechanismRatio = gearRatio;
    launcherHoodConfig.Feedback.RotorToSensorRatio = 1.0;

    // Hardware limit switches - NONE
    // launcherHoodConfig.HardwareLimitSwitch.*

    // Motion Magic settings - fused CANcoder affects all feedback constants by the gearRatio
    launcherHoodConfig.MotionMagic.MotionMagicCruiseVelocity = 25.0;      // Rotations / second
    launcherHoodConfig.MotionMagic.MotionMagicAcceleration = 100.0;       // Rotations / second ^ 2
    launcherHoodConfig.MotionMagic.MotionMagicJerk = 1000.0;              // Rotations / second ^ 3

    // Motor output settings
    launcherHoodConfig.MotorOutput.DutyCycleNeutralDeadband = 0.001;    // Percentage
    launcherHoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    launcherHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Open Loop settings
    // launcherHoodConfig.OpenLoopRamps.*                               // Seconds to ramp

    // Slot settings - remote/fused CANcoder affects all feedback constants by the gearRatio
    // kG = (0.40 + 0.25) / 2
    // kS = (0.40 - 0.25) / 2
    launcherHoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Feedforward: Mechanism is an arm and needs cosine
    launcherHoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    launcherHoodConfig.Slot0.kS = 0.07;                                 // Feedforward: Voltage or duty cycle to overcome static friction
    launcherHoodConfig.Slot0.kG = 0.0;                                  // Feedforward: Voltage or duty cycle to overcome gravity (arbitrary feedforward)
    launcherHoodConfig.Slot0.kV = 0.1129;                               // Feedforward: Voltage or duty cycle per requested RPS (velocity modes)

    // NOTE: Motion Magic settings are scaled by gear ration when using a FusecCANCoder
    launcherHoodConfig.Slot0.kP = 2.0 * gearRatio;                                  // Feedback: Voltage or duty cycle per velocity unit (velocity modes)
    launcherHoodConfig.Slot0.kI = 0.0 * gearRatio;                                  // Feedback: Voltage or duty cycle per accumulated unit
    launcherHoodConfig.Slot0.kD = 0.1 * gearRatio;                                  // Feedback: Voltage or duty cycle per unit of acceleration unit (velocity modes)

    // Software limit switches
    // launcherHoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = min;  // Rotations
    // launcherHoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // launcherHoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = max;  // Rotations
    // launcherHoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    return launcherHoodConfig;
  }

  /****************************************************************************
   * 
   * TBD CANRange detector
   */
  public static CANrangeConfiguration xxxCANRangeConfig( )
  {
    CANrangeConfiguration crConfig = new CANrangeConfiguration( );

    crConfig.ProximityParams.ProximityThreshold = 0.1;                  // Proximity distance in meters (about 8 inches)
    crConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    return crConfig;
  }

  // Swerve module configs are built into swerve subsystem

}
