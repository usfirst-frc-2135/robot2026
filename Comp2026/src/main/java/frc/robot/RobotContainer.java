// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoScore1A;
import frc.robot.autos.AutoScore1B;
import frc.robot.autos.AutoScore2A;
import frc.robot.autos.AutoScore2B;
import frc.robot.autos.AutoTest;
import frc.robot.commands.AcquireFuel;
import frc.robot.commands.ClimbTower;
import frc.robot.commands.ExpelFuel;
import frc.robot.commands.LaunchFuel;
import frc.robot.commands.LogCommand;
import frc.robot.commands.PrepareToClimb;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.StopIntaking;
import frc.robot.commands.StopLaunching;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.HID;
import frc.robot.lib.LED;
import frc.robot.lib.MatchState;
import frc.robot.lib.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Power;
import frc.robot.subsystems.Telemetry;

/****************************************************************************
 * 
 * This class is where the bulk of the robot is declared. Since Command-based is a "declarative"
 * paradigm, very little robot logic should actually be handled in the Robot periodic methods (other
 * than the scheduler calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer
{
  private final boolean                               m_macOSXSim     = false;  // Enables Mac OS X controller compatibility in simulation

  // Gamepad controllers
  private static final CommandXboxController          m_driverPad     = new CommandXboxController(Constants.kDriverPadPort);
  private static final CommandXboxController          m_operatorPad   = new CommandXboxController(Constants.kOperatorPadPort);

  private static final LinearVelocity                 kMaxSpeed       = TunerConstants.kSpeedAt12Volts;       // Maximum top speed
  private static final AngularVelocity                kMaxAngularRate = RotationsPerSecond.of(1.0); // Max 1.0 rot per second
  private static final double                         kSlowSwerve     = 0.30;                                 // Throttle max swerve speeds for finer control
  private static final double                         kHeadingKp      = 6.0;
  private static final double                         kHeadingKi      = 0.0;
  private static final double                         kHeadingKd      = 0.0;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric            drive           = new SwerveRequest.FieldCentric( ) //
      .withDeadband(kMaxSpeed.times(0.1))                 //
      .withRotationalDeadband(kMaxAngularRate.times(0.1)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);       // We want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake        brake           = new SwerveRequest.SwerveDriveBrake( );
  private final SwerveRequest.FieldCentricFacingAngle facing          = new SwerveRequest.FieldCentricFacingAngle( )  //
      .withDeadband(kMaxSpeed.times(0.1))                 //
      .withRotationalDeadband(kMaxAngularRate.times(0.1)) //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);       // We want field-centric driving in open loop
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt           point           = new SwerveRequest.PointWheelsAt( );
  private final SwerveRequest.RobotCentric            aim             = new SwerveRequest.RobotCentric( );
  private final SwerveRequest.Idle                    idle            = new SwerveRequest.Idle( );
  @SuppressWarnings("unused")
  private final SwerveRequest.RobotCentric            forwardStraight = new SwerveRequest.RobotCentric( )     //
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry                             logger          = new Telemetry(kMaxSpeed.in(MetersPerSecond));

  // Utility classes for LED and HID controls
  private final LED                                   m_led           = new LED( );
  private final HID                                   m_hid           = new HID(m_driverPad.getHID( ), m_operatorPad.getHID( ));
  private final Vision                                m_vision        = new Vision( );
  private MatchState                                  m_matchState;

  // The robot's shared subsystems
  private final Power                                 m_power         = new Power( );

  // These subsystems may use LED or vision and must be created afterward
  private final CommandSwerveDrivetrain               m_drivetrain    = TunerConstants.createDrivetrain( );
  private final Intake                                m_intake        = new Intake( );
  private final Hopper                                m_hopper        = new Hopper( );
  private final Kicker                                m_kicker        = new Kicker( );
  private final Launcher                              m_launcher      = new Launcher( );
  // private final Climber                               m_climberLeft   = new Climber("Left", "CL-", false);
  private final Climber                               m_climberRight  = new Climber("Right", "CR-", false);

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
    AUTOSCORE2B         // 
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
  private SendableChooser<AutoChooser>  m_autoChooser  = new SendableChooser<>( );
  /** Dashboard chooser for starting pose selection */
  private SendableChooser<StartPose>    m_startChooser = new SendableChooser<>( );

  /**
   * Hash map of autonomous option relations to auto filenames
   * 
   * @param key
   *          the auto option and pose selected
   * @param value
   *          the auto filename associated with the key
   */
  private final HashMap<String, String> autoMap        = new HashMap<>(Map.ofEntries(                 //
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START1.toString( ), "Start1_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START2.toString( ), "Start2_Stop"),
      Map.entry(AutoChooser.AUTOSTOP.toString( ) + StartPose.START3.toString( ), "STart3_Stop"),

      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1"),
      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START2.toString( ), "Start2_L2A_L2A"),
      Map.entry(AutoChooser.AUTOSCORE1A.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3"),

      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1"),
      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START2.toString( ), "Start2_L2A_L2A"),
      Map.entry(AutoChooser.AUTOSCORE1B.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3"),

      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1_NZ1A_L1"),
      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START2.toString( ), "Start2_L2A_L2A_OP_L2B"),
      Map.entry(AutoChooser.AUTOSCORE2A.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3_NZ3A_L3"),

      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START1.toString( ), "Start1_NZ1_L1_D_L2A"),
      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START2.toString( ), "Start2_L2A_L2A_OP_L2B"),
      Map.entry(AutoChooser.AUTOSCORE2B.toString( ) + StartPose.START3.toString( ), "Start3_NZ3_L3_OP_L2B"),

      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START1.toString( ), "Start1_T1"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START2.toString( ), "Start2_T2"),
      Map.entry(AutoChooser.AUTOTEST.toString( ) + StartPose.START3.toString( ), "Start3_T3")       // 
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

    m_matchState = new MatchState(m_hid, m_led);
    addDashboardWidgets( );     // Add some dashboard widgets for commands
    configureBindings( );       // Configure game controller buttons and triggers
    initDefaultCommands( );     // Initialize subsystem default commands

    // Add periodic calls for non-subsystem classes
    robot.addPeriodic(( ) -> m_hid.periodic( ), Seconds.of(0.020));
    robot.addPeriodic(( ) -> m_led.periodic( ), Seconds.of(0.020));
    robot.addPeriodic(( ) -> m_matchState.periodic( ), Seconds.of(0.020));

    Robot.timeMarker("robotContainer: after default commands");
  }

  /****************************************************************************
   * 
   * Callbacks used by dashboard autonomous choosers to reload when an onChange event occurs
   */
  private void updateAutoChooserCallback(AutoChooser option)
  {
    Robot.reloadAutomousCommand(option.toString( ));
  }

  private void updateStartChooserCallback(StartPose option)
  {
    Robot.reloadAutomousCommand(option.toString( ));
  }

  /****************************************************************************
   * 
   * Create general dashboard widgets for commands and subsystems
   */
  private void addDashboardWidgets( )
  {
    // Network tables publisher objects

    // Build autonomous chooser objects on dashboard and fill the options
    SmartDashboard.putData("AutoMode", m_autoChooser);
    SmartDashboard.putData("StartPosition", m_startChooser);
    SmartDashboard.putNumber("AutoDelay", 0.0);

    // Configure autonomous sendable chooser
    m_autoChooser.setDefaultOption("0 - AutoStop", AutoChooser.AUTOSTOP);
    m_autoChooser.addOption("1 - AutoScore1A", AutoChooser.AUTOSCORE1A);
    m_autoChooser.addOption("2 - AutoScore1B", AutoChooser.AUTOSCORE1B);
    m_autoChooser.addOption("3 - AutoScore2A", AutoChooser.AUTOSCORE2A);
    m_autoChooser.addOption("4 - AutoScore2B", AutoChooser.AUTOSCORE2B);
    m_autoChooser.addOption("9 - AutoTest", AutoChooser.AUTOTEST);
    m_autoChooser.onChange(this::updateAutoChooserCallback);

    // Configure starting pose sendable chooser
    m_startChooser.setDefaultOption("START1", StartPose.START1);
    m_startChooser.addOption("START2", StartPose.START2);
    m_startChooser.addOption("START3", StartPose.START3);
    m_startChooser.onChange(this::updateStartChooserCallback);

    // Add a button that allows running autonomous commands in teleop
    SmartDashboard.putData("AutoChooserRun", new InstantCommand(( ) ->
    {
      if (m_autoCommand.isScheduled( ))
      {
        m_autoCommand.cancel( );
      }

      if ((m_autoCommand = getAutonomousCommand( )) != null)
      {
        CommandScheduler.getInstance( ).schedule(m_autoCommand);
      }
    }));

    // Add main command scheduler to dashboard

    SmartDashboard.putData(CommandScheduler.getInstance( ));

    // Add command groups to dashboard

  }

  /****************************************************************************
   * 
   * Define button-command mappings. Triggers are created and bound to the desired commands.
   */
  private void configureBindings( )
  {
    ///////////////////////////////////////////////////////
    //
    // Driver Controller Assignments
    //
    // Driver - A, B, X, Y
    // 
    m_driverPad.a( ).whileTrue(m_drivetrain.applyRequest(( ) -> aim       //
        .withVelocityX(m_vision.rangeProportional(kMaxSpeed))             //
        .withVelocityY(0)                                    //
        .withRotationalRate(m_vision.aimProportional(kMaxAngularRate))));
    m_driverPad.b( ).onTrue(new LogCommand("driverPad", "B"));

    m_driverPad.x( ).onTrue(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getHID( ).getXButtonPressed( ))));
    m_driverPad.x( ).onFalse(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getHID( ).getXButtonPressed( ))));
    m_driverPad.y( ).whileTrue(getSlowSwerveCommand( )); // Note: left lower paddle!

    //
    // Driver - Bumpers, start, back
    //
    m_driverPad.leftBumper( ).onTrue(new ExpelFuel(m_intake, m_hopper));
    m_driverPad.leftBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));
    m_driverPad.rightBumper( ).onTrue(new AcquireFuel(m_intake, m_hopper));
    m_driverPad.rightBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));

    m_driverPad.back( ).whileTrue(m_drivetrain.applyRequest(( ) -> brake));                             // aka View button
    m_driverPad.start( ).onTrue(m_drivetrain.runOnce(( ) -> m_drivetrain.seedFieldCentric( )));         // aka Menu button

    //
    // Driver - POV buttons
    //
    // m_driverPad.pov(0).onTrue(new PrepareToClimb(m_intake, m_climberRight));
    // m_driverPad.pov(180).onTrue(new ClimbTower(m_climberRight));
    m_driverPad.pov(0).onTrue(new PrepareToClimb(m_intake, m_climberRight));
    m_driverPad.pov(90).onTrue(new ClimbTower(m_climberRight));
    m_driverPad.pov(180).whileTrue(m_drivetrain.applyRequest(( ) -> facing  //
        .withVelocityX(kMaxSpeed.times(-m_driverPad.getLeftY( )))                 //
        .withVelocityY(kMaxSpeed.times(-m_driverPad.getLeftX( )))                 //
        .withTargetDirection(Rotation2d.fromDegrees(-180.0))));
    m_driverPad.pov(270).onTrue(new LogCommand("driverPad", "POV 270"));

    //
    // Driver Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_driverPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new RetractIntake(m_intake, m_hopper));
    m_driverPad.leftTrigger(Constants.kTriggerThreshold).onFalse(new StopIntaking(m_intake, m_hopper));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new LaunchFuel(m_intake, m_hopper, m_kicker, m_launcher));
    m_driverPad.rightTrigger(Constants.kTriggerThreshold).onFalse(new StopLaunching(m_hopper, m_kicker, m_launcher));

    m_driverPad.leftStick( ).onTrue(new LogCommand("driverPad", "left stick"));
    m_driverPad.rightStick( ).onTrue(new LogCommand("driverPad", "right stick"));

    ///////////////////////////////////////////////////////
    //
    // Operator Controller Assignments
    //
    // Operator - A, B, X, Y
    //
    m_operatorPad.a( ).onTrue(Commands.runOnce(( ) -> m_launcher.decrementTeleopRPM( )));
    m_operatorPad.b( ).onTrue(Commands.runOnce(( ) -> m_launcher.incrementTeleopRPM( )));

    m_operatorPad.x( ).onTrue(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getHID( ).getXButtonPressed( ))));
    m_operatorPad.x( ).onFalse(Commands.runOnce(( ) -> m_hopper.setPulseMode(m_operatorPad.getHID( ).getXButtonPressed( ))));
    m_operatorPad.y( ).onTrue(new LogCommand("operatorPad", "Y"));

    //
    // Operator - Bumpers, start, back
    //
    m_operatorPad.leftBumper( ).onTrue(new ExpelFuel(m_intake, m_hopper));
    m_operatorPad.leftBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));
    m_operatorPad.rightBumper( ).onTrue(new AcquireFuel(m_intake, m_hopper));
    m_operatorPad.rightBumper( ).onFalse(new StopIntaking(m_intake, m_hopper));

    m_operatorPad.back( ).toggleOnTrue(m_climberRight.getJoystickCommand(( ) -> getClimberAxis( )));   // aka View button
    m_operatorPad.start( ).toggleOnTrue(m_intake.getJoystickCommand(( ) -> getIntakeRotaryAxis( )));  // aka Menu button

    //
    // Operator - POV buttons
    //
    m_operatorPad.pov(0).onTrue(new PrepareToClimb(m_intake, m_climberRight));
    m_operatorPad.pov(90).onTrue(new ClimbTower(m_climberRight));
    m_operatorPad.pov(180).onTrue(new LogCommand("operatorPad", "POV:180"));
    m_operatorPad.pov(270).onTrue(new LogCommand("operatorPad", "POV:270"));

    //
    // Operator Left/Right Trigger
    //
    // Xbox enums { leftX = 0, leftY = 1, leftTrigger = 2, rightTrigger = 3, rightX = 4, rightY = 5}
    // Xbox on MacOS { leftX = 0, leftY = 1, rightX = 2, rightY = 3, leftTrigger = 5, rightTrigger = 4}
    //
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onTrue(new RetractIntake(m_intake, m_hopper));
    m_operatorPad.leftTrigger(Constants.kTriggerThreshold).onFalse(new StopIntaking(m_intake, m_hopper));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onTrue(new LaunchFuel(m_intake, m_hopper, m_kicker, m_launcher));
    m_operatorPad.rightTrigger(Constants.kTriggerThreshold).onFalse(new StopLaunching(m_hopper, m_kicker, m_launcher));

    m_operatorPad.leftStick( ).toggleOnTrue(new LogCommand("operPad", "left stick"));
    m_operatorPad.rightStick( ).toggleOnTrue(new LogCommand("operPad", "right stick"));
  }

  /****************************************************************************
   * 
   * Initialize default commands for these subsystems
   */
  private void initDefaultCommands( )
  {
    m_launcher.setDefaultCommand(m_launcher.getLauncherPrimedCommand());

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
              .withRotationalRate(kMaxAngularRate.times(-m_driverPad.getLeftTriggerAxis( )))          // Drive counterclockwise with negative X (left)
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
  void resetOdometryToInitialPose(PathPlannerPath initialPath)
  {
    if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
    {
      initialPath = initialPath.flipPath( );
    }

    // Set field centric robot position to start of auto sequence
    try
    {
      Optional<Pose2d> startPose = initialPath.getStartingHolonomicPose( );
      m_drivetrain.resetPoseAndLimelight(startPose.get( ));
      DataLogManager.log(String.format("getAuto: starting pose %s", startPose));
    }
    catch (Exception nullException)
    {
      DataLogManager.log(String.format("getAuto: ERROR! - starting pose is missing"));
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
    AutoChooser autoOption = m_autoChooser.getSelected( );
    StartPose startOption = m_startChooser.getSelected( );
    double delay = SmartDashboard.getNumber("AutoDelay", 0.0);

    // Cancel any autos that were already running

    if (m_autoCommand != null)
    {
      if (m_autoCommand.isScheduled( ))
      {
        m_autoCommand.cancel( );
      }
      m_autoCommand = null;
    }

    // Get auto name using created key

    String autoKey = autoOption.toString( ) + startOption.toString( );
    String autoName = autoMap.get(autoKey);
    DataLogManager.log(String.format("========================================================================"));
    DataLogManager.log(String.format("getAuto: autoKey: %s  autoName: %s", autoKey, autoName));
    DataLogManager.log(String.format("========================================================================"));

    // Get list of paths within the auto file for all autos except AUTOSTOP

    try
    {
      m_ppPathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    }
    catch (ParseException | IOException e)
    {
      DataLogManager.log(String.format("getAuto: ERROR - parse or IO exception when reading the auto file"));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }

    if (m_ppPathList.isEmpty( ))
    {
      DataLogManager.log(String.format("getAuto: ERROR - auto path list is empty"));
      return m_autoCommand = m_drivetrain.applyRequest(( ) -> idle);
    }

    DataLogManager.log(String.format("getAuto: %s contains %s paths in list", autoName, m_ppPathList.size( )));

    // {
    //   // Debug only: print states of first path
    //   List<PathPlannerTrajectory.State> states = m_initialPath.getTrajectory(new ChassisSpeeds( ), new Rotation2d( )).getStates( );
    //   for (int i = 0; i < states.size( ); i++)
    //     DataLogManager.log(String.format("autoCommand: Auto path state: (%d) %s", i, states.get(i).getTargetHolonomicPose( )));
    // }

    // Create the correct base command and pass the path list

    switch (autoOption)
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

    DataLogManager.log(String.format("getAuto: autoMode %s (%s)", autoKey, m_autoCommand.getName( )));

    // Build a new sequential command from the base command that allows for a delay and handles odometry

    // Update robot pose to where we want immediately so it displays correctly in dashboard
    resetOdometryToInitialPose(m_ppPathList.get(0));

    // Build the autonomous command to run
    if (autoOption != AutoChooser.AUTOSTOP)
    {
      m_autoCommand = new SequentialCommandGroup(                                                       //
          new InstantCommand(( ) -> Robot.timeMarker("AutoStart")),                                 //
          new InstantCommand(( ) ->      // Update pose again right before we run the command
          {
            resetOdometryToInitialPose(m_ppPathList.get(0));
          }, m_drivetrain),                                                                             //
          new LogCommand("Autodelay", String.format("Delaying %.1f seconds ...", delay)), //
          new WaitCommand(delay),                                                                       //
          m_autoCommand,                                                                                //
          new InstantCommand(( ) -> Robot.timeMarker("AutoEnd"))                                    //
      );
    }

    return m_autoCommand;
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
    m_climberRight.initialize( );
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void autoInit( )
  {
    m_vision.run( );

    m_launcher.initAutonomousRPM( );
  }

  /****************************************************************************
   * 
   * Called during teleopInit to start any needed commands
   */
  public void teleopInit( )
  {
    m_vision.run( );

    m_launcher.initTeleopRPM( );
  }

  /****************************************************************************
   * 
   * Called when user button is pressed - place subsystem fault dumps here
   */
  public void printAllFaults( )
  {
    m_led.printFaults( );
    m_power.printFaults( );

    m_intake.printFaults( );
    m_hopper.printFaults( );
    m_kicker.printFaults( );
    m_launcher.printFaults( );
    m_climberRight.printFaults( );
  }
}
