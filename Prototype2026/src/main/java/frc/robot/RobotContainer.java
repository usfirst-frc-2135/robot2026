package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer
{
  private static final boolean m_isComp    = detectRobot( );
  private static final double  kEncoderCPR = 4096;

  private static enum ControlMode
  {
    kStopped, kFixedSpeed, kJoystickControl, kClosedLoop
  }

  public static boolean isComp( )
  {
    return m_isComp;
  }

  private static boolean detectRobot( )
  {
    // Detect which robot/RoboRIO
    String serialNum = System.getenv("serialnum");
    String robotName = "UNKNOWN";
    boolean isComp = false;

    DataLogManager.log(String.format("robotContainer: RoboRIO SN: %s", serialNum));
    if (serialNum == null)
      robotName = "SIMULATION";
    else if (serialNum.equals("032B1F7E"))
    {
      isComp = true;
      robotName = "COMPETITION (A)";
    }
    else if (serialNum.equals("03260A3A"))
    {
      isComp = false;
      robotName = "PRACTICE/BETA (B)";
    }
    DataLogManager.log(String.format("robotContainer: Detected the %s robot (RoboRIO)!", robotName));

    return isComp;
  }

  // In a real robot, the servo actuator would be declared within a subsystem and not here
  private final Servo m_actuator = new Servo(0);

  private void AddDashboardWidgets( )
  {
    SmartDashboard.putData("Actuator_IN_Open", Commands.runOnce(( ) -> m_actuator.setSpeed(1.0)));
    SmartDashboard.putData("Actuator_OUT_Close", Commands.runOnce(( ) -> m_actuator.setSpeed(-1.0)));
  }

  public RobotContainer( )
  {
    
    AddDashboardWidgets( ); // Add dashboard widgets for commands

    configureButtonBindings( );       // Configure game controller buttons

    initDefaultCommands( );           // Initialize subsystem default commands

    Robot.timeMarker("robotContainer: after default commands");
  }

  private void configureButtonBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    //  --- Normal button definitions ---
    //
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim       //
        .withVelocityX(m_vision.rangeProportional(kMaxSpeed))             //
        .withVelocityY(0)                                    //
        .withRotationalRate(m_vision.aimProportional(kMaxAngularRate))));
    m_driverPad.b( ).onTrue(new LogCommand("driverPad", "B")); // drive to stage right
    m_driverPad.x( ).onTrue(new LogCommand("driverPad", "X")); // drive to stage left
    m_driverPad.y( ).onTrue(new LogCommand("driverPad", "Y")); // drive to stage center
    //
    //  --- SysId button definitions ---
    //
    // Run SysId routines when holding A, B and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverPad.a( ).and(m_driverPad.y( )).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverPad.a( ).and(m_driverPad.x( )).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverPad.b( ).and(m_driverPad.y( )).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverPad.b( ).and(m_driverPad.x( )).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kAmpPose)); // drive to amp
    m_driverPad.rightBumper( ).onTrue(new AcquireNote(m_intake, m_led, m_hid));
    m_driverPad.rightBumper( ).onFalse(new RetractIntake(m_intake, m_led, m_hid));
    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                             // aka View button
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldCentric( )));         // aka Menu button

    //
    // Driver - POV buttons
    //
    m_driverPad.pov(0).whileTrue(m_drivetrain.applyRequest(( ) -> facing    //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(0.0))));
    m_driverPad.pov(90).whileTrue(m_drivetrain.applyRequest(( ) -> facing   //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(270.0))));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(180.0))));
    m_driverPad.pov(270).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(90.0))));

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold)
        .whileTrue(m_drivetrain.drivePathtoPose(m_drivetrain, VIConsts.kSpeakerPose));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreSpeaker(m_shooter, m_intake, m_led));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(m_shooter.getShooterScoreCommand( ));
    m_operatorPad.b( ).onTrue(m_shooter.getShooterStopCommand( ));
    m_operatorPad.x( ).onTrue(new PassNote(m_shooter, m_intake, m_led));
    m_operatorPad.x( ).onFalse(m_shooter.getShooterScoreCommand( ));
    m_operatorPad.y( ).onTrue(new ExpelNote(m_intake, m_led));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new HandoffToFeeder(m_intake, m_feeder, m_led));
    m_operatorPad.rightBumper( ).onTrue(new AcquireNote(m_intake, m_led, m_hid));
    m_operatorPad.rightBumper( ).onFalse(new RetractIntake(m_intake, m_led, m_hid));
    m_operatorPad.back( ).toggleOnTrue(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));                                     // aka View button
    m_operatorPad.start( ).onTrue(new InstantCommand(m_vision::rotateCameraStreamMode).ignoringDisable(true));  // aka Menu button

    //
    // Operator - POV buttons
    //


    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreSpeaker(m_shooter, m_intake, m_led));

    m_operatorPad.leftStick( ).toggleOnTrue(m_feeder.getJoystickCommand(( ) -> getFeederAxis( )));
    m_operatorPad.rightStick( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
  }

}
