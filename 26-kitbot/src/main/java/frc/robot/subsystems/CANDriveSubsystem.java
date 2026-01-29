// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase
{
  private final WPI_TalonSRX      leftFront  = new WPI_TalonSRX(LEFT_LEADER_ID);
  private final WPI_TalonSRX      leftBack   = new WPI_TalonSRX(LEFT_FOLLOWER_ID);
  private final WPI_TalonSRX      rightFront = new WPI_TalonSRX(RIGHT_LEADER_ID);
  private final WPI_TalonSRX      rightBack  = new WPI_TalonSRX(RIGHT_FOLLOWER_ID);
  private final DifferentialDrive drive      = new DifferentialDrive(leftFront, rightFront);

  public CANDriveSubsystem( )
  {
    // create brushed motors for drive

    // set up differential drive class

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    leftFront.configFactoryDefault( );
    leftBack.configFactoryDefault( );
    rightFront.configFactoryDefault( );
    rightBack.configFactoryDefault( );

    // rightSide.setInverted(true);

    // this.leftFront.setSafetyEnabled(true);
    // this.leftFront.setExpiration(250.0);
    // this.rightFront.setSafetyEnabled(true);
    // this.rightFront.setExpiration(250.0);

    TalonSRXConfiguration config = new TalonSRXConfiguration( );
    config.voltageCompSaturation = 12.0;//max voltage
    config.continuousCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
    leftFront.configAllSettings(config);
    rightFront.configAllSettings(config);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    leftFront.setNeutralMode(NeutralMode.Coast);
    rightFront.setNeutralMode(NeutralMode.Coast);
    rightFront.setInverted(true);
    rightBack.setInverted(true);

  }

  @Override
  public void periodic( )
  {}

  public void driveArcade(double xSpeed, double zRotation)
  {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
