package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer
{
  private static final CommandXboxController m_driverPad = new CommandXboxController(0);
  // private static final CommandXboxController m_operatorPad = new CommandXboxController(1);
  private static double                      m_timeMark  = Timer.getFPGATimestamp( );
  private final Intake                       m_intake    = new Intake( );
  private final Shooter                      m_shooter   = new Shooter( );

  // In a real robot, the servo actuator would be declared within a subsystem and not here
  private final Servo                        m_actuator  = new Servo(0);

  private void AddDashboardWidgets( )
  {
    SmartDashboard.putData("Actuator_IN_Open", Commands.runOnce(( ) -> m_actuator.setSpeed(1.0)));
    SmartDashboard.putData("Actuator_OUT_Close", Commands.runOnce(( ) -> m_actuator.setSpeed(-1.0)));
  }

  public RobotContainer( )
  {

    AddDashboardWidgets( ); // Add dashboard widgets for commands

    configureButtonBindings( ); // Configure game controller buttons

    initDefaultCommands( ); // Initialize subsystem default commands

    timeMarker("robotContainer: after default commands");
  }

  private void configureButtonBindings( )
  {
    m_driverPad.a( ).onTrue(m_shooter.getShooterScoreCommand( ));
    m_driverPad.b( ).onTrue(m_shooter.getShooterStopCommand( ));

    m_driverPad.x( ).onTrue(m_intake.getIntakeOnCommand( ));
    m_driverPad.y( ).onTrue(m_intake.getIntakeOffCommand( ));

    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    //
    // --- Normal button definitions ---
    //

    // m_driverPad.b( ).onTrue(new LogCommand("driverPad", "B")); // drive to stage right
    // m_driverPad.x( ).onTrue(new LogCommand("driverPad", "X")); // drive to stage left
    // m_driverPad.y( ).onTrue(new LogCommand("driverPad", "Y")); // drive to stage center
    //
    // --- SysId button definitions ---
    //
    // Run SysId routines when holding A, B and X/Y.
    // Note that each routine should be run exactly once in a single log.

    // m_driverPad.a( ).and(m_driverPad.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverPad.b( ).and(m_driverPad.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverPad.b( ).and(m_driverPad.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    //
    // Driver - Bumpers, start, back
    //
    // m_driverPad.rightBumper( ).onTrue(new AcquireNote(m_intake));

    //
    // Driver - POV buttons
    //

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    // m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new ScoreSpeaker(m_shooter, m_intake, m_led));

    // m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    // m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    // m_operatorPad.a( ).onTrue(m_shooter.getShooterScoreCommand( ));
    // m_operatorPad.b( ).onTrue(m_shooter.getShooterStopCommand( ));
    // m_operatorPad.x( ).onTrue(new PassNote(m_shooter, m_intake, m_led));
    // m_operatorPad.x( ).onFalse(m_shooter.getShooterScoreCommand( ));
    // m_operatorPad.y( ).onTrue(new ExpelNote(m_intake, m_led));

    //
    // Operator - Bumpers, start, back
    //
    // m_operatorPad.leftBumper( ).onTrue(new HandoffToFeeder(m_intake, m_feeder, m_led));
    // m_operatorPad.rightBumper( ).onTrue(new AcquireNote(m_intake, m_led, m_hid));
    // m_operatorPad.rightBumper( ).onFalse(new RetractIntake(m_intake, m_led, m_hid));

    //
    // Operator - POV buttons
    //
    // m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new
    // ScoreSpeaker(m_shooter, m_intake, m_led));

    // m_operatorPad.rightStick( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeAxis( )));
  }

  private void initDefaultCommands( )
  {
    // Default command - Motion Magic hold
    // m_intake.setDefaultCommand(m_intake.getHoldPositionCommand(INRollerMode.HOLD, m_intake::getCurrentPosition));

    // Default command - manual mode
    // m_intake.setDefaultCommand(m_intake.getJoystickCommand(( ) -> getIntakeAxis()));
  }

  public static void timeMarker(String msg)
  {
    double now = Timer.getFPGATimestamp( );

    DataLogManager.log(String.format("***** TimeMarker ***** absolute: %.3f relative: %.3f - %s", now, now - m_timeMark, msg));

    m_timeMark = now;
  }

}
