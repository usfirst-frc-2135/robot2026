// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase
{
  private final WPI_TalonSRX feederRoller;
  private final WPI_TalonSRX intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem( )
  {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new WPI_TalonSRX(FEEDER_MOTOR_ID);
    feederRoller.configFactoryDefault( );
    TalonSRXConfiguration feederConfig = new TalonSRXConfiguration( );
    feederConfig.continuousCurrentLimit = (FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configAllSettings(feederConfig);

    intakeLauncherRoller = new WPI_TalonSRX(INTAKE_LAUNCHER_MOTOR_ID);
    intakeLauncherRoller.configFactoryDefault( );
    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    TalonSRXConfiguration launcherConfig = new TalonSRXConfiguration( );
    launcherConfig.continuousCurrentLimit = (LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configAllSettings(launcherConfig);

    intakeLauncherRoller.setInverted(true);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double voltage)
  {
    intakeLauncherRoller.setVoltage(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double voltage)
  {
    feederRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop( )
  {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
  }
}
