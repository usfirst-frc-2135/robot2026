// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.DataLogManager;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import bline.field.FlippingUtil;
import bline.path.Path;
import first.robot.Constants.KKConsts;
import first.robot.Constants.KKConsts.KKRollerMode;
import first.robot.autos.AutoScore1A;
import first.robot.autos.AutoScore1B;
import first.robot.autos.AutoScore2A;
import first.robot.autos.AutoScore2B;
import first.robot.autos.AutoScore3A;
import first.robot.autos.AutoTest;
import first.robot.commands.AcquireFuel;
import first.robot.commands.ClimbTower;
import first.robot.commands.ExpelFuel;
import first.robot.commands.LaunchFuel;
import first.robot.commands.LogCommand;
import first.robot.commands.PrepareToClimb;
import first.robot.commands.RetractIntake;
import first.robot.commands.StopIntaking;
import first.robot.commands.StopLaunching;
import first.robot.generated.TunerConstants;
import first.robot.lib.HID;
import first.robot.lib.LED;
import first.robot.lib.MatchPhase;
import first.robot.lib.Vision;
import first.robot.subsystems.Climber;
import first.robot.subsystems.Drivetrain;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;
import first.robot.subsystems.Power;
import first.robot.subsystems.SwerveTelemetry;

/****************************************************************************
 * 
 * This class is where the bulk of the robot is declared. Since Command-based is a "declarative"
 * paradigm, very little robot logic should actually be handled in the Robot periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                               m_macOSXSim       = false;  // Enables Mac OS X controller compatibility in simulation

  // Autonomous health/status observability for pit and development debugging
  private static final String                         kAutoHealthState  = "AutoHealthState";
  private static final String                         kAutoHealthReason = "AutoHealthReason";
  private static final String                         kAutoHealthKey    = "AutoHealthAutoKey";
  private static final String                         kAutoHealthName   = "AutoHealthAutoName";
  private static final String                         kAutoHealthCmd    = "AutoHealthCommand";
  private static final String                         kAutoHealthTime   = "AutoHealthTimestamp";

  // Gamepad controllers
  private static final CommandXboxController          m_driverPad       = new CommandXboxController(Constants.kDriverPad);
  private static final CommandXboxController          m_operatorPad     = new CommandXboxController(Constants.kOperatorPad);

  private static final LinearVelocity                 kMaxSpeed         = TunerConstants.kSpeedAt12Volts;       // Maximum top speed
  private static final AngularVelocity                kMaxAngularRate   = RotationsPerSecond.of(1.0); // Max 1.0 rot per second
  private static final double                         kSlowSwerve       = 0.30;                                 // Throttle max swerve speeds for finer control
  private static final double                         kHeadingKp        = 6.0;
  private static final double                         kHeadingKi        = 0.0;
  private static final double                         kHeadingKd        = 0.0;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric            drive             = new SwerveRequest.FieldCentric( ) //
      .withDeadband(kMaxSpeed.times(0.1))                 //
      .withRotationalDeadband(kMaxAngularRate.times(0.1)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);       // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake             = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing            = new SwerveRequest.FieldCentricFacingAngle( )  //
      .withDeadband(kMaxSpeed.times(0.1))                 //
      .withRotationalDeadband(kMaxAngularRate.times(0.1)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);       // We want field-centric driving in open loop
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt           point             = new SwerveRequest.PointWheelsAt( );
  private final SwerveRequest.RobotCentric            aim               = new SwerveRequest.RobotCentric( );
  private final SwerveRequest.Idle                    idle              = new SwerveRequest.Idle( );
  @SuppressWarnings("unused")
  private final SwerveRequest.RobotCentric            forwardStraight   = new SwerveRequest.RobotCentric( )     //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveTelemetry                       logger            = new SwerveTelemetry(kMaxSpeed.in(MetersPerSecond));

  // Utility classes for LED and HID controls
  private final LED                                   m_led             = new LED( );
  private final HID                                   m_hid             = new HID(m_driverPad.getHID( ), m_operatorPad.getHID( ));
  private final Vision                                m_vision          = new Vision( );
  private MatchPhase                                  m_matchPhase;

  // The robot's shared subsystems
  private final Power                                 m_power           = new Power( );

  // These subsystems may use LED or vision and must be created afterward
  private final Drivetrain                            m_drivetrain      = new Drivetrain( );
  private final Intake                                m_intake          = new Intake( );
  private final Hopper                                m_hopper          = new Hopper( );
  private final Kicker                                m_kicker          = new Kicker( );
  private final Launcher                              m_launcher        = new Launcher( );
  private final Climber                               m_climber         = new Climber( );

  // Selected autonomous command
  private Command                                     m_autoCommand;  // Selected autonomous command
  private List<PathPlannerPath>                       m_ppPathList;   // Path list for the selected auto

  /**
   * Chooser options for autonomous commands - all starting from poses 1-3
   */
  private enum AutoChooser
  {
    AUTOSTOP,           // AutoStop - sit still, do nothing
    AUTOTEST,           // Run a selected test auto
    AUTOSCORE1A,        // One cycle of scoring fuel - 2 paths (out to neutral zone, back to score)
    AUTOSCORE1B,        // One cycle of scoring fuel - 1 big looping path that returns to scoring position
    AUTOSCORE2A,        // Two cycles of scoring fuel - 3 paths (out to neutral zone, back to launch, looping path back to score)
    AUTOSCORE2B,        // PP: Two cycles of scoring fuel - 2 paths (out to neutral zone and back to launch, looping path back to score)
    AUTOSCORE3A         // BLine: Two cycles of scoring fuel - 2 paths (out to neutral zone and back to launch, looping path back to score)
  }

  /**
   * Chooser options for autonomous starting pose to select pose 1-3 (driver perspective)
   */
  private enum StartPose
  {
    START1, // Starting pose 1 - leftmost aligned with trench 
    START2, // Starting pose 2 - middle aligned with hub       
    START3  // Starting pose 3 - rightmost aligned with trench
  }

  /** Dashboard chooser for auto option selection */
  private Selectable<AutoChooser>                  m_autoChooser   = new Selectable<>( );
  /** Dashboard chooser for starting pose selection */
  private Selectable<StartPose>                    m_startChooser  = new Selectable<>( );
  /** Dashboard widget for auto delay time */
  private double                                   m_autoDelayPrev = 0.0;
  private TunableDouble                            m_autoDelay     = Tunables.addDouble("AutoDelay", m_autoDelayPrev);

  /**
   * Hash map of autonomous option relations to auto filenames
   * 
   * @param key
   *          the auto option and pose selected
   * @param value
   *          the auto filename associated with the key
   */
  private final HashMap<String, String>            ppAutoMap       = new HashMap<>(Map.ofEntries(    //
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START1.toString( ), "Start1_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START2.toString( ), "Start2_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START3.toString( ), "Start3_Stop"),

      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1"),
      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START2.toString( ), "Start2_L2A_L2A"),
      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3"),

      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START1.toString( ), "Start1_NZ1A_NZ1B_NZ1C_L1"),
      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START2.toString( ), "Start2_L2B_L2B"),
      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START3.toString( ), "Start3_NZ3A_NZ3B_NZ3C_L3"),

      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1_NZ1C_L1"),
      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START2.toString( ), "Start2_L2B_L2B_L2B"),
      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3_NZ3C_L3"),

      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1_NZ1_L1"),
      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START2.toString( ), "Start2_L2A"),
      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3_NZ3_L3"),

      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START1.toString( ), "Start1_S1_L1_L1_L1"),
      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START2.toString( ), "Start2_S2_L2_L2_L2"),
      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START3.toString( ), "Start3_S3_L3_L3_L3"),

      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START1.toString( ), "Start1_T1"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START2.toString( ), "Start2_T2"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START3.toString( ), "Start3_T3") // 
  ));

  private final HashMap<String, ArrayList<String>> bLineAutoMap    = new HashMap<>(Map.ofEntries(    //
      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START1.toString( ),
          new ArrayList<>(Arrays.asList("S1_L1", "L1_L1"))),
      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START2.toString( ),
          new ArrayList<>(Arrays.asList("S1_L1", "L2_L2"))),
      Map.entry(AutoChooser.AUTOSCORE3A.toString( ) + StartPose.START3.toString( ),
          new ArrayList<>(Arrays.asList("S3_L3", "L3_L3"))) //
  ));

  /****************************************************************************
   * 
   * The main container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Robot robot)
  {
    Robot.timeMarker("robotContainer: before heading controller and field layout");

    facing.HeadingController = new PhoenixPIDController(kHeadingKp, kHeadingKi, kHeadingKd);    // Swerve steer PID for facing swerve request
    facing.HeadingController.enableContinuousInput(-180.0, 180.0);

    m_matchPhase = new MatchPhase(m_hid, m_led);
    addDashboardWidgets( );     // Add some dashboard widgets for commands
    configureBindings( );       // Configure game controller buttons and triggers
    initDefaultCommands( );     // Initialize subsystem default commands

    Robot.timeMarker("robotContainer: after default commands");
  }

  /****************************************************************************
   * 
   * Callbacks used by dashboard autonomous choosers to reload when an onChange event occurs
   */
  private void updateAutoChooserCallback(AutoChooser option)
  {
    Robot.reloadAutonomousCommand((option != null) ? option.toString( ) : "AutoChooser:null");
  }

  private void updateStartChooserCallback(StartPose option)
  {
    Robot.reloadAutonomousCommand((option != null) ? option.toString( ) : "StartChooser:null");
  }

  public boolean autoDelayIsChanged( )
  {
    double autoDelay = m_autoDelay.get( );

    if (m_autoDelayPrev != autoDelay)
    {
      m_autoDelayPrev = autoDelay;
      return true;
    }

    return false;
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addDashboardWidgets( )
  {
    // Network tables publisher objects

    // Build autonomous chooser objects on dashboard and fill the options
    publishAutoHealthStatus("INIT", "RobotContainerInit", "none", "none", "none");

    // Configure autonomous sendable chooser
    m_autoChooser.addDefault("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.add("1 - AutoScore1A", AutoChooser.AUTOSCORE1A);
    m_autoChooser.add("2 - AutoScore1B", AutoChooser.AUTOSCORE1B);
    m_autoChooser.add("3 - AutoScore2A", AutoChooser.AUTOSCORE2A);
    m_autoChooser.add("4 - AutoScore2B", AutoChooser.AUTOSCORE2B);
    m_autoChooser.add("5 - AutoScore3A", AutoChooser.AUTOSCORE3A);
    m_autoChooser.add("9 - AutoTest", AutoChooser.AUTOTEST);
    m_autoChooser.onChange(this::updateAutoChooserCallback);
    Tunables.publish("AutoMode", m_autoChooser);

    // Configure starting pose sendable chooser
    m_startChooser.addDefault("START1", StartPose.START1);
    m_startChooser.add("START2", StartPose.START2);
    m_startChooser.add("START3", StartPose.START3);
    m_startChooser.onChange(this::updateStartChooserCallback);
    Tunables.publish("AutoStart", m_startChooser);

    // Publish the autonomous delay
    Tunables.addDouble("AutoDelay", 0.0);

    // Add a button that allows running autonomous commands in teleop
    Tunables.publish("AutoChooserRun", new InstantCommand(( ) ->
    {
      if (m_autoCommand != null && m_autoCommand.isScheduled( ))
      {
        m_autoCommand.cancel( );
      }

      m_autoCommand = getAutonomousCommand( );  // Must always return a valid command, even if it's just AutoStop
      CommandScheduler.getInstance( ).schedule(m_autoCommand);
    }));

    Tunables.publish("ResetFaults", new InstantCommand(( ) ->
    {
      m_led.printFaults( );
      m_power.printFaults( );

      // m_drivetrain.printFaults( );
      m_intake.printFaults( );
      m_hopper.printFaults( );
      m_kicker.printFaults( );
      m_launcher.printFaults( );
      m_climber.printFaults( );
    }));

  }

  /****************************************************************************
   * 
   * Define button-command mappings. Triggers are created and bound to the desired commands.
   */
  private void configureBindings( )
  {
    ////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    // 
    m_driverPad.a( ).whileTrue(m_drivetrain.getAutoAlignToHubCommand( ));
    m_driverPad.b( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim       //
        .withVelocityX(m_vision.rangeProportional(kMaxSpeed))             //
        .withVelocityY(0)                                    //
        .withRotationalRate(m_vision.aimProportional(kMaxAngularRate))));

    m_driverPad.x( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));
    m_driverPad.y( ).whileTrue(getSlowSwerveCommand( )); // Note: left lower paddle!

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).onTrue(new ExpelFuel(m_intake, m_hopper, m_kicker));
    m_driverPad.leftBumper( ).onFalse(new ParallelCommandGroup(  //
        new StopIntaking(m_intake, m_hopper),                 //
        m_kicker.getRollerModeCommand(KKRollerMode.STOP)      //
    ));
    m_driverPad.rightBumper( ).onTrue(new SequentialCommandGroup( //
        m_kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),//
        m_launcher.getLauncherPrimedCommand( ), //
        new AcquireFuel(m_intake, m_hopper)));//
    m_driverPad.rightBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));

    m_driverPad.view( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                     // aka View button
    m_driverPad.menu( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldCentric( )));  // aka Menu button

    //
    // Driver - POV buttons
    //
    m_driverPad.dpadUp( ).onTrue(new PrepareToClimb(m_intake, m_climber));
    m_driverPad.dpadRight( ).onTrue(new ClimbTower(m_climber));
    m_driverPad.dpadDown( ).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-180.0))));
    m_driverPad.dpadLeft( ).onTrue(new LogCommand("driverPad", "POV 270"));

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new RetractIntake(m_intake, m_hopper));
    // m_driverPad.leftTrigger(Constants.kTriggerThreshold).onFalse(new StopIntaking(m_intake, m_hopper));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new LaunchFuel(m_intake, m_hopper, m_kicker, m_launcher));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onFalse(new StopLaunching(m_intake, m_hopper, m_kicker, m_launcher));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(Commands.runOnce(( ) -> m_launcher.decrementTeleopRPM( )));
    m_operatorPad.b( ).onTrue(Commands.runOnce(( ) -> m_launcher.incrementTeleopRPM( )));

    m_operatorPad.x( )
        .onTrue(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getController( ).getXButtonPressed( ))));
    m_operatorPad.x( )
        .onFalse(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getController( ).getXButtonPressed( ))));
    m_operatorPad.y( ).onTrue(m_launcher.getLauncherStopCommand( ));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new ExpelFuel(m_intake, m_hopper, m_kicker));
    m_operatorPad.leftBumper( ).onFalse(new ParallelCommandGroup(  //
        new StopIntaking(m_intake, m_hopper),                 //
        m_kicker.getRollerModeCommand(KKRollerMode.STOP)      //
    ));
    m_operatorPad.rightBumper( ).onTrue(new SequentialCommandGroup( //
        m_kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),//
        m_launcher.getLauncherPrimedCommand( ), //
        new AcquireFuel(m_intake, m_hopper)));//
    m_operatorPad.rightBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));

    m_operatorPad.view( ).toggleOnTrue(m_climber.getJoystickCommand(( ) -> getClimberAxis( )));   // aka View button
    m_operatorPad.menu( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeRotaryAxis( )));  // aka Menu button

    //
    // Operator - POV buttons
    //
    m_operatorPad.dpadUp( ).onTrue(new PrepareToClimb(m_intake, m_climber));
    m_operatorPad.dpadRight( ).onTrue(new ClimbTower(m_climber));
    m_operatorPad.dpadDown( ).onTrue(new LogCommand("operatorPad", "POV:180"));
    m_operatorPad.dpadLeft( ).onTrue(new LogCommand("operatorPad", "POV:270"));

    //
    // Operator Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new RetractIntake(m_intake, m_hopper));
    // m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onFalse(new StopIntaking(m_intake, m_hopper));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new LaunchFuel(m_intake, m_hopper, m_kicker, m_launcher));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onFalse(new StopLaunching(m_intake, m_hopper, m_kicker, m_launcher));

    m_operatorPad.leftStick( ).toggleOnTrue(new LogCommand("operPad", "left stick"));
    m_operatorPad.rightStick( ).toggleOnTrue(new LogCommand("operPad", "right stick"));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    if (!m_macOSXSim)
    {
      m_drivetrain.setDefaultCommand(                                                                 // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                                      //
              .withVelocityX(kMaxSpeed.times(                                                         // Apply deadband to joystick value
                  MathUtil.applyDeadband(-m_driverPad.getLeftY( ), 0.15, 1.0))) // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(                                                         // Apply deadband to joystick value
                  MathUtil.applyDeadband(-m_driverPad.getLeftX( ), 0.15, 1.0))) // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getRightX( )))                   // Drive counterclockwise with negative X (left)
          )                                                                                           //
              .withName("CommandSwerveDrivetrain"));
    }
    else // When using simulation on MacOS X, XBox controllers need to be re-mapped due to an Apple bug
    {
      m_drivetrain.setDefaultCommand(                                                                 // Drivetrain will execute this command periodically
          m_drivetrain.applyRequest(( ) -> drive                                                      //
              .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                               // Drive forward with negative Y (forward)
              .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                               // Drive left with negative X (left)
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getLeftTrigger( )))          // Drive counterclockwise with negative X (left)
          )                                                                                           //
              .withName("CommandSwerveDrivetrain"));
    }

    // Idle while the robot is disabled. This ensures the configured neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled( ).whileTrue(m_drivetrain.applyRequest(( ) -> idle).ignoringDisable(true));

    // Note: Only one default command can be active per subsystem--use the manual modes during bring-up

    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  /****************************************************************************
   * 
   * Reset odometery to initial pose in the first autonomous path
   */
  void resetOdometryToInitialPose(Optional<Pose2d> initialPose)
  {
    if (initialPose == null)
    {
      DataLogManager.log("getAuto: ERROR! - initial pose container is null, cannot reset odometry");
      return;
    }

    // Set field centric robot position to start of auto sequence
    try
    {
      if (initialPose.isPresent( ))
      {
        m_drivetrain.resetPoseAndLimelight(initialPose.get( ));
        DataLogManager.log(String.format("getAuto: starting pose %s", initialPose));
      }
      else
      {
        DataLogManager.log("getAuto: ERROR! - starting pose is missing");
      }
    }
    catch (Exception e)
    {
      DataLogManager.log(String.format("getAuto: ERROR! - failed to read/reset starting pose (%s)", e.toString( )));
    }

  }

  /****************************************************************************
   * 
   * Use this to pass the autonomous command to the main Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand( )
  {
    // NOTE: This method is intentionally called from Robot.disabledPeriodic() to preload auto.
    // It may parse/read path files and construct command graphs, which can take noticeable time.
    // Keep autonomousInit() schedule-only to avoid delayed robot response at auto start.
    //
    // IMPORTANT POLICY: Never return null from this method. Always return a valid command,
    // falling back to AutoStop/idle on any error.
    AutoSelectionContext selection = resolveAutoSelectionContext( );  // Get auto name using created key

    DataLogManager.log(String.format("========================================================================"));
    DataLogManager.log(String.format("getAuto: autoKey: %s  autoName: %s", selection.autoKey, selection.autoName));
    DataLogManager.log(String.format("========================================================================"));

    String fallbackReason = evaluateAutoMapFallbackReason(selection.autoName);
    if (fallbackReason != null)
    {
      DataLogManager.log(String.format("getAuto: ERROR - no auto mapped for key: %s", selection.autoKey));
      return m_autoCommand = getAutoStopFallback(fallbackReason, selection.autoKey, "missing");
    }

    if (selection.autoOption != AutoChooser.AUTOSCORE3A)
    {
      return buildPathPlannerAutonomousCommand(selection);
    }

    // BLine autonomous option
    return buildBLineAutonomousCommand(selection);
  }

  /****************************************************************************
   * 
   * Build a valid fallback autonomous command that keeps the robot still.
   *
   * IMPORTANT POLICY: Autonomous command selection must never return null.
   * On any error condition, return an explicit AutoStop/idle command and log why.
   */
  private Command getAutoStopFallback(String reason, String autoKey, String autoName)
  {
    publishAutoHealthStatus("FALLBACK", reason, autoKey, autoName, "AutoStop-" + reason);
    DataLogManager.log(String.format("getAuto: FALLBACK to AutoStop - %s (key=%s name=%s)", reason, autoKey, autoName));
    return m_drivetrain.applyRequest(( ) -> idle).withName("AutoStop-" + reason);
  }

  /****************************************************************************
   *
   * Publish autonomous health status for pit and development diagnostics.
   */
  private void publishAutoHealthStatus(String state, String reason, String autoKey, String autoName, String commandName)
  {
    Telemetry.log(kAutoHealthState, state);
    Telemetry.log(kAutoHealthReason, reason);
    Telemetry.log(kAutoHealthKey, autoKey);
    Telemetry.log(kAutoHealthName, autoName);
    Telemetry.log(kAutoHealthCmd, commandName);
    Telemetry.log(kAutoHealthTime, org.wpilib.system.Timer.getTimestamp( ));
  }

  private Command buildPreRunCommand(Runnable preRunHook)
  {
    Command preRunCommand;
    if (preRunHook != null)
    {
      preRunCommand = new InstantCommand(preRunHook, m_drivetrain);
    }
    else
    {
      preRunCommand = new InstantCommand(( ) ->
      {
        // No pre-run action for this autonomous command.
      }, m_drivetrain);
    }
    return preRunCommand;
  }

  /****************************************************************************
   *
   * Wrap a base autonomous command with common start/end markers and delay behavior.
   * Optionally run a pre-run hook immediately before the delay and base command.
   */
  private Command buildFullAutoCommand(Command baseAutoCommand, double delay, Runnable preRunHook)
  {
    if (baseAutoCommand == null)
    {
      DataLogManager.log("getAuto: WARNING - baseAutoCommand was null in wrapper; substituting AutoStop");
      baseAutoCommand = m_drivetrain.applyRequest(( ) -> idle).withName("AutoStop-WrappedNull");
    }

    Command preRunCommand = buildPreRunCommand(preRunHook);

    return new SequentialCommandGroup(                                                                //
        new InstantCommand(( ) -> Robot.timeMarker("AutoStart")),                                  //
        preRunCommand,                                                                                //
        new LogCommand("Autodelay", String.format("Delaying %.1f seconds ...", delay)), //
        new WaitCommand(delay),                                                                       //
        baseAutoCommand,                                                                              //
        new InstantCommand(( ) -> Robot.timeMarker("AutoEnd"))                                     //
    );
  }

  /****************************************************************************
   *
   * Finalize autonomous command with shared status publish, preview pose reset,
   * and optional wrapped execution flow.
   */
  private Command finalizeAutonomousCommand(AutoSelectionContext selection, Command baseAutoCommand, Optional<Pose2d> initialPose)
  {
    m_autoCommand = baseAutoCommand;

    DataLogManager.log(String.format("getAuto: autoMode %s (%s)", selection.autoKey, m_autoCommand.getName( )));
    publishAutoHealthStatus("OK", "Ready", selection.autoKey, selection.autoName, m_autoCommand.getName( ));

    // Update robot pose to where we want immediately so it displays correctly in dashboard
    resetOdometryToInitialPose(initialPose);

    // Build the autonomous command to run
    if (selection.autoOption != AutoChooser.AUTOSTOP)
    {
      m_autoCommand = buildFullAutoCommand(m_autoCommand, selection.delay, ( ) -> resetOdometryToInitialPose(initialPose));
    }

    return m_autoCommand;
  }

  /****************************************************************************
   *
   * Shared autonomous selection context resolved from chooser state and dashboard delay.
   */
  private static final class AutoSelectionContext
  {
    final AutoChooser autoOption;
    final double      delay;
    final String      autoKey;
    final String      autoName;

    AutoSelectionContext(AutoChooser autoOption, double delay, String autoKey, String autoName)
    {
      this.autoOption = autoOption;
      this.delay = delay;
      this.autoKey = autoKey;
      this.autoName = autoName;
    }
  }

  /****************************************************************************
   *
   * Resolve chooser selections into a shared autonomous selection context.
   */
  private AutoSelectionContext resolveAutoSelectionContext( )
  {
    AutoChooser autoOption = m_autoChooser.getSelected( );
    StartPose startOption = m_startChooser.getSelected( );

    if (autoOption == null)
    {
      autoOption = AutoChooser.AUTOSTOP;
      DataLogManager.log("getAuto: WARNING - AutoMode selection was null, defaulting to AUTOSTOP");
    }

    if (startOption == null)
    {
      startOption = StartPose.START1;
      DataLogManager.log("getAuto: WARNING - AutoStart selection was null, defaulting to START1");
    }

    String autoKey = autoOption.toString( ) + startOption.toString( );
    String autoName = ppAutoMap.get(autoKey);
    double delay = m_autoDelay.get( );

    return new AutoSelectionContext(autoOption, delay, autoKey, autoName);
  }

  /****************************************************************************
   *
   * Build autonomous command for PathPlanner-backed autos.
   */
  private Command buildPathPlannerAutonomousCommand(AutoSelectionContext selection)
  {
    // Get list of paths within the auto file for all autos except AUTOSTOP

    try
    {
      m_ppPathList = loadPathGroupFromAutoFile(selection.autoName);
    }
    catch (ParseException | IOException e)
    {
      DataLogManager.log(String.format("getAuto: ERROR - parse or IO exception when reading the auto file (%s)", e.toString( )));
      return m_autoCommand = getAutoStopFallback("PathPlannerPathLoadError", selection.autoKey, selection.autoName);
    }

    String loadedPathFallbackReason = evaluateLoadedPathFallbackReason(m_ppPathList);
    if (loadedPathFallbackReason != null)
    {
      DataLogManager.log(String.format("getAuto: ERROR - auto path list is empty"));
      return m_autoCommand = getAutoStopFallback(loadedPathFallbackReason, selection.autoKey, selection.autoName);
    }

    DataLogManager.log(String.format("getAuto: %s contains %s paths in list", selection.autoName, m_ppPathList.size( )));

    // Create the correct base command and pass the path list

    switch (selection.autoOption)
    {
      default :
      case AUTOSTOP :
        m_autoCommand = m_drivetrain.applyRequest(( ) -> idle).withName("AutoStop");
        break;
      case AUTOTEST :
        m_autoCommand = new AutoTest(m_ppPathList, m_drivetrain);
        break;
      case AUTOSCORE1A :
        m_autoCommand = new AutoScore1A(m_ppPathList, m_drivetrain, m_intake, m_hopper, m_kicker, m_launcher);
        break;
      case AUTOSCORE1B :
        m_autoCommand = new AutoScore1B(m_ppPathList, m_drivetrain, m_intake, m_hopper, m_kicker, m_launcher);
        break;
      case AUTOSCORE2A :
        m_autoCommand = new AutoScore2A(m_ppPathList, m_drivetrain, m_intake, m_hopper, m_kicker, m_launcher);
        break;
      case AUTOSCORE2B :
        m_autoCommand = new AutoScore2B(m_ppPathList, m_drivetrain, m_intake, m_hopper, m_kicker, m_launcher);
        break;
    }

    try
    {
      Optional<Pose2d> initialPose = getPathPlannerInitialPose(m_ppPathList);
      return finalizeAutonomousCommand(selection, m_autoCommand, initialPose);
    }
    catch (RuntimeException e)
    {
      DataLogManager.log(String.format("getAuto: ERROR - PathPlanner build failed (%s)", e.toString( )));
      return m_autoCommand = getAutoStopFallback("PathPlannerBuildError", selection.autoKey, selection.autoName);
    }
  }

  /****************************************************************************
   *
   * Build autonomous command for BLine-backed autos.
   */
  private Command buildBLineAutonomousCommand(AutoSelectionContext selection)
  {
    List<String> bLinePathNames = new ArrayList<String>( );
    List<Path> bLinePaths = new ArrayList<Path>( );
    Optional<Pose2d> initialPose;

    // Get list of paths within the auto file for all autos except AUTOSTOP

    bLinePathNames = bLineAutoMap.get(selection.autoKey);

    if (bLinePathNames == null || bLinePathNames.isEmpty( ))
    {
      DataLogManager.log(String.format("getAuto: ERROR - BLine path name list is empty for key %s", selection.autoKey));
      return m_autoCommand = getAutoStopFallback("BLinePathListEmpty", selection.autoKey, selection.autoName);
    }

    for (String pathName : bLinePathNames)
    {
      try
      {
        Path path = new Path(pathName);
        bLinePaths.add(path);
      }
      catch (Exception e)
      {
        DataLogManager.log(String.format("getAuto: ERROR - exception when reading the BLine path file (%s)", e.toString( )));
        return m_autoCommand = getAutoStopFallback("BLinePathLoadError", selection.autoKey, selection.autoName);
      }
    }

    if (bLinePaths.isEmpty( ))
    {
      DataLogManager.log(String.format("getAuto: ERROR - no BLine paths were built for key %s", selection.autoKey));
      return m_autoCommand = getAutoStopFallback("BLinePathListEmpty", selection.autoKey, selection.autoName);
    }

    initialPose = Optional.ofNullable(FlippingUtil.flipFieldPose(bLinePaths.get(0).getStartPose( )));

    DataLogManager.log(String.format("getAuto: %s contains %s paths in list", selection.autoName, bLinePaths.size( )));

    try
    {
      m_autoCommand = new AutoScore3A(bLinePaths, m_drivetrain, m_intake, m_hopper, m_kicker, m_launcher);
      return finalizeAutonomousCommand(selection, m_autoCommand, initialPose);
    }
    catch (RuntimeException e)
    {
      DataLogManager.log(String.format("getAuto: ERROR - BLine build failed (%s)", e.toString( )));
      return m_autoCommand = getAutoStopFallback(evaluateBLineBuildFallbackReason(e), selection.autoKey, selection.autoName);
    }
  }

  /****************************************************************************
   * 
   * Use to slow down swerve drivetrain to 30 percent max speed. Drivetrain will execute this command
   * when invoked
   */
  private Command getSlowSwerveCommand( )
  {
    return m_drivetrain.applyRequest(( ) -> drive                                                 // 
        .withVelocityX(kMaxSpeed.times(kSlowSwerve).times(-m_driverPad.getLeftY( )))              // Drive forward with negative Y (forward)
        .withVelocityY(kMaxSpeed.times(kSlowSwerve).times(-m_driverPad.getLeftX( )))              // Drive left with negative X (left)
        .withRotationalRate(kMaxAngularRate.times(kSlowSwerve).times(-m_driverPad.getRightX( )))  // Drive counterclockwise with negative X (left)
    )                                                                                             //
        .withName("CommandSlowSwerveDrivetrain");
  }

  /****************************************************************************
   * 
   * Gamepad joystick axis interfaces
   */
  private double getIntakeRotaryAxis( )
  {
    return m_operatorPad.getRightX( );
  }

  /****************************************************************************
   * 
   * Gamepad joystick axis interfaces
   */
  private double getClimberAxis( )
  {
    return -m_operatorPad.getLeftY( );
  }

  /****************************************************************************
   * 
   * Called by disabledInit - place subsystem initializations here
   */
  public void disabledInit( )
  {
    m_hid.initialize( );
    m_led.initialize( );
    m_power.initialize( );

    m_vision.initialize( );

    m_intake.initialize( );
    m_hopper.initialize( );
    m_kicker.initialize( );
    m_launcher.initialize( );

    m_climber.initialize( );
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void autoInit( )
  {
    m_vision.run( );

    m_launcher.initAutonomousRPM( );

    CommandScheduler.getInstance( ).schedule(m_launcher.getLauncherPrimedCommand( ));
    CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit( )
  {
    m_vision.run( );

    m_launcher.initTeleopRPM( );
    m_matchPhase.teleopInit( );

    CommandScheduler.getInstance( ).schedule(m_launcher.getLauncherPrimedCommand( ));
    CommandScheduler.getInstance( ).schedule(m_climber.getCalibrateCommand( ));
  }

  /****************************************************************************
   * 
   * Called during robotPeriodic to call our libraries periodic methods
   */
  public void robotPeriodic( )
  {
    // Add periodic calls for non-subsystem classes
    m_hid.periodic( );
    m_led.periodic( );
    m_matchPhase.periodic( );
  }

  /****************************************************************************
   *
   * Test seam for loading PathPlanner auto path groups.
   */
  public List<PathPlannerPath> loadPathGroupFromAutoFile(String autoName) throws ParseException, IOException
  {
    return PathPlannerAuto.getPathGroupFromAutoFile(autoName);
  }

  /****************************************************************************
   *
   * Test seam for extracting initial pose from PathPlanner path list.
   */
  public Optional<Pose2d> getPathPlannerInitialPose(List<PathPlannerPath> pathList)
  {
    if (pathList == null || pathList.isEmpty( ))
    {
      return Optional.empty( );
    }

    PathPlannerPath initialPath = pathList.get(0);
    if (initialPath == null)
    {
      return Optional.empty( );
    }

    if (MatchState.getAlliance( ).orElse(Alliance.BLUE) == Alliance.RED)
    {
      initialPath = initialPath.flipPath( );
    }

    return initialPath.getStartingHolonomicPose( );
  }

  /****************************************************************************
   *
   * Test seam: evaluate map-lookup fallback reason without touching hardware.
   *
   * @return null when no fallback is needed; otherwise fallback reason token.
   */
  public static String evaluateAutoMapFallbackReason(String autoName)
  {
    if (autoName == null)
    {
      return "AutoMapMissing";
    }
    return null;
  }

  /****************************************************************************
   *
   * Test seam: evaluate loaded-path fallback reason without touching hardware.
   *
   * @return null when no fallback is needed; otherwise fallback reason token.
   */
  public static String evaluateLoadedPathFallbackReason(List<PathPlannerPath> pathList)
  {
    if (pathList == null || pathList.isEmpty( ))
    {
      return "PPPathListEmpty";
    }
    return null;
  }

  /****************************************************************************
   *
   * Test seam: evaluate BLine build failure without touching hardware.
   *
   * @return null when no fallback is needed; otherwise fallback reason token.
   */
  public static String evaluateBLineBuildFallbackReason(Throwable error)
  {
    if (error != null)
    {
      return "BLineBuildError";
    }
    return null;
  }

}
