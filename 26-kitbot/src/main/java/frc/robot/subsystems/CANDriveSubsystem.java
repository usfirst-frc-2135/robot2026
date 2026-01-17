// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase
{
  private final WPI_TalonSRX      leftFront = new WPI_TalonSRX(1);
  private final WPI_TalonSRX      leftBack;
  private final WPI_TalonSRX      rightFront;
  private final WPI_TalonSRX      rightBack;
  private final DifferentialDrive drive;

  public CANDriveSubsystem( )
  {
    this.leftFront.configFactoryDefault( );
    this.leftBack = new WPI_TalonSRX(2);
    this.leftBack.configFactoryDefault( );
    this.rightFront = new WPI_TalonSRX(3);
    this.rightFront.configFactoryDefault( );
    this.rightBack = new WPI_TalonSRX(4);
    this.rightBack.configFactoryDefault( );
    this.leftBack.set(ControlMode.Follower, (double) this.leftFront.getDeviceID( ));
    this.rightBack.set(ControlMode.Follower, (double) this.rightFront.getDeviceID( ));
    this.drive = new DifferentialDrive(this.leftFront, this.rightFront);
    this.leftFront.setSafetyEnabled(true);
    this.leftFront.setExpiration(250.0);
    this.leftBack.setSafetyEnabled(true);
    this.leftBack.setExpiration(250.0);
    this.rightFront.setSafetyEnabled(true);
    this.rightFront.setExpiration(250.0);
    this.rightBack.setSafetyEnabled(true);
    this.rightBack.setExpiration(250.0);
    this.rightFront.setInverted(true);
    TalonSRXConfiguration config = new TalonSRXConfiguration( );
    config.peakCurrentLimit = 60;
    config.continuousCurrentLimit = 40;
    config.peakCurrentDuration = 1000;
  }

  public void periodic( )
  {}

  public void driveArcade(double xSpeed, double zRotation)
  {
    this.drive.arcadeDrive(xSpeed, zRotation);
  }
}
