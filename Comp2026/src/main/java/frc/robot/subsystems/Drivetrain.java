//
// Swerve Subystem - command-based swerve subsystem
//
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.LogCommand;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.Vision;
import frc.robot.lib.BLine.FollowPath;

/**
 * Class that extends the TunerX generated CommandSwerveDrivetrain class and extends it with
 * additional functionality for the robot.
 */
public class Drivetrain extends CommandSwerveDrivetrain
{
  /***********************************************************************************/
  /***********************************************************************************/

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds( );

  /* Get the network tables instance */
  private final NetworkTableInstance           inst                   = NetworkTableInstance.getDefault( );
  private final NetworkTable                   kSwerveTable           = inst.getTable("swerve");

  /* Limelight network table entries for field localization */
  private final DoubleArrayPublisher           kLLPoseFront           =
      kSwerveTable.getDoubleArrayTopic("llPose-front").publish( );
  private final DoubleArrayPublisher           kLLPoseBack            =
      kSwerveTable.getDoubleArrayTopic("llPose-back").publish( );
  private final BooleanPublisher               m_frontUpdate          = kSwerveTable.getBooleanTopic("FrontCam").publish( );
  private final BooleanPublisher               m_backUpdate           = kSwerveTable.getBooleanTopic("BackCam").publish( );
  private final BooleanEntry                   m_useLimelightEntry    =
      kSwerveTable.getBooleanTopic("useLimelight").getEntry(false);

  /* Limelight enable control */
  private final boolean                        m_useLimelight         = true; // Global enable for limelight vision updates

  /* Swerve auto-align constants */
  private static final double                  kAimingKp              = 0.012;
  private static final double                  kDrivingKp             = 0.6;
  private static final double                  kOptimalDistance       = Units.inchesToMeters(141.0);

  /* Robot set pose entry and auto-align controls */
  private final DoubleArrayEntry               m_setPoseEntry         =
      kSwerveTable.getDoubleArrayTopic("setPose").getEntry(new double[3]);
  private DoublePublisher                      m_rangePub             = kSwerveTable.getDoubleTopic("AlignRange").publish( );
  private DoublePublisher                      m_aimPub               = kSwerveTable.getDoubleTopic("AlignAim").publish( );

  /* Pose entry from Telemetry used for auto-align to a pose */
  private final NetworkTable                   kDriveStateTable       = inst.getTable("DriveState");
  private final StructSubscriber<Pose2d>       m_driveStatePose       =
      kDriveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d( ));

  // Class member variables

  /* Limelight tracking filters for vision updates */
  private MedianFilter                         m_frontFilter          = new MedianFilter(5);
  private MedianFilter                         m_backFilter           = new MedianFilter(5);

  /* Module distances for swerve kinematics calibration */
  private double[ ]                            m_moduleDistances      = new double[4];

  /* Swerve setpoint generator variables */
  private SwerveSetpointGenerator              m_setpointGenerator;
  private SwerveSetpoint                       m_previousSetpoint;

  /* Hub center for auto-align */
  private Translation2d                        m_hubCenter            = new Translation2d( );

  /* Robot pathToPose constraints */
  private final PathConstraints                kPathFindConstraints   = new PathConstraints( // 
      3.5,            // kMaxVelocityMps
      3.5,      // kMaxAccelerationMpsSq
      2.0 * Math.PI,                 // kMaxAngularSpeedRadiansPerSecond
      2.0 * Math.PI                  // kMaxAngularSpeedRadiansPerSecondSquared
  );

  private static FollowPath.Builder            pathBuilder;

  /**
   * Constructs the Drivetrain and adds dashboard widgets and network table entries for the
   * drivetrain.
   */
  public Drivetrain( )
  {
    super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
        TunerConstants.BackRight);

    configureAutoBuilder( );
    initDashboard( );
  }

  /**
   * Configures the PathPlanner AutoBuilder and BLine PathBuilder for this drivetrain. This allows the
   * robot to follow paths generated by PathPlanner or BLine.
   */
  private void configureAutoBuilder( )
  {
    /**
     * PathPlanner AutoBuilder configuration.
     */
    try
    {
      var config = RobotConfig.fromGUISettings( );
      AutoBuilder.configure(( ) -> getState( ).Pose,  // Supplier of current robot pose
          this::resetPose,                            // Consumer for seeding pose against auto
          ( ) -> getState( ).Speeds,                  // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          // (speeds) -> driveRobotRelative(speeds),     // FUTURE: swerve setpoint generator sample code (supposed to be more accurate)
          (speeds,
              feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons( ))
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons( ))),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          ( ) -> DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
      );

      // FUTURE:  Setpoint generator defined and untested for path following, but not used in the AutoBuilder above.

      m_setpointGenerator = new SwerveSetpointGenerator(  //
          config,                                         // The robot configuration used for generating trajectories and running path following.
          Units.rotationsToRadians(10.0)        // The max rotation velocity of a swerve module in rads/s. Should be stored in your Constants file
      );

      // Initialize the previous setpoint to the robot's current speeds and module states
      ChassisSpeeds currentSpeeds = this.getState( ).Speeds;                // Method to get current robot-relative chassis speeds
      SwerveModuleState[ ] currentStates = this.getState( ).ModuleStates;   // Method to get the current swerve module states
      m_previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(config.numModules));
    }
    catch (Exception ex)
    {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace( ));
    }

    /**
     * BLine PathBuilder - an alternative way to generate and follow paths from PathPlanner.
     */
    pathBuilder = new FollowPath.Builder(             // Create a new FollowPath.Builder
        this,                                         // Drive subsystem to use for path following
        ( ) -> getState( ).Pose,                      // Supplier of current robot pose
        ( ) -> getState( ).Speeds,                    // Supplier of current robot speeds
        this::driveRobotRelative,                     // Consumer of ChassisSpeeds to drive the robot
        new PIDController(2.0, 0.0, 0.0),   // PID constants for translation
        new PIDController(2.0, 0.0, 0.0),   // PID constants for rotation
        new PIDController(0.2, 0.0, 0.0)    // PID constants for steering
    )                                                 //
        .withDefaultShouldFlip( )                           // Use default behavior for flipping paths based on alliance color
        .withTRatioBasedTranslationHandoffs(true);  // Use T-ratio based translation handoffs
  }

  /**
   * Exposes the BLine PathBuilder for this drivetrain. This allows paths to be generated which can be
   * followed using the BLine library, which is an alternative to PathPlanner path following.
   */
  public FollowPath.Builder getPathBuilder( )
  {
    return pathBuilder;
  }

  /**
   * Returns a command that applies the specified control (drive) request to this swerve drivetrain.
   *
   * @param request
   *          Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
  {
    return super.applyRequest(requestSupplier);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction
   *          Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
  {
    return super.sysIdQuasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by
   * {@link #m_sysIdRoutineToApply}.
   *
   * @param direction
   *          Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction)
  {
    return super.sysIdDynamic(direction);
  }

  /**
   * Drivetrain periodic which adds vision updates to the base CommandSwerveDrivetrain
   */
  @Override
  public void periodic( )
  {
    super.periodic( );

    double front = (visionUpdate(Constants.kLLFrontName, kLLPoseFront)) ? 1.0 : 0.0;
    m_frontUpdate.set(m_frontFilter.calculate(front) > 0.5);

    double back = (visionUpdate(Constants.kLLBackName, kLLPoseBack)) ? 1.0 : 0.0;
    m_backUpdate.set(m_backFilter.calculate(back) > 0.5);
  }

  /**
   * Add dashboard widgets and network table entries
   */
  private void initDashboard( )
  {
    m_setPoseEntry.set(new double[3]);

    // Adds a button for resetting the robot pose to the current pose of the drivetrain. This is useful for testing and debugging.
    SmartDashboard.putData("SetPose", getResetPoseCommand( ));

    // SmartDashboard.putData("AlignToPosePID", getAlignToPosePIDCommand( ));
    // SmartDashboard.putData("AlignToPosePPFollow", new DeferredCommand(( ) -> getPoseAlignPPFollowCommand( ), Set.of(this)));
    // SmartDashboard.putData("AlignToPosePPFind", new DeferredCommand(( ) -> getAlignToPosePPFindCommand( ), Set.of(this)));
    m_useLimelightEntry.set(m_useLimelight);
  }

  /****************************************************************************
   * 
   * Limelight MegaTag code for vision processing
   * 
   * @param limelightName
   *          Name of the Limelight to use
   * @param poseArray
   *          Publisher for the pose estimate
   * @return true if the vision update is valid, false otherwise
   */
  private boolean visionUpdate(String limelightName, DoubleArrayPublisher poseArray)
  {
    boolean useMegaTag2 = true;           // set to false to use MegaTag1
    boolean doRejectUpdate = false;

    // MegaTag1 processing

    if (useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

      if (mt1 == null)
      {
        doRejectUpdate = true;
      }
      else
      {
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
          if (mt1.rawFiducials[0].ambiguity > 0.7)
          {
            doRejectUpdate = true;
          }
          if (mt1.rawFiducials[0].distToCamera > 3.0)
          {
            doRejectUpdate = true;
          }
        }
        if (mt1.tagCount == 0)
        {
          doRejectUpdate = true;
        }
      }

      if (!doRejectUpdate)
      {
        double[ ] array =
        {
            mt1.pose.getX( ), mt1.pose.getY( ), mt1.pose.getRotation( ).getDegrees( )
        };
        poseArray.set(array);

        if (m_useLimelightEntry.get( ))
        {
          setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
          super.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }
      }
    }

    // MegaTag2 processing

    else if (useMegaTag2 == true)
    {
      // FUTURE: Add angular velocity to the LimelightHelpers.SetRobotOrientation call below.
      // Pigeon2 pigeon = getPigeon2( );
      // double yawRate = pigeon.getAngularVelocityZWorld( ).getValueAsDouble( );
      // LimelightHelpers.SetRobotOrientation(limelightName, pigeon.getYaw( ).getValueAsDouble( ), yawRate,
      //         pigeon.getPitch( ).getValueAsDouble( ), 0, pigeon.getRoll( ).getValueAsDouble( ), 0);

      LimelightHelpers.SetRobotOrientation(limelightName, getState( ).Pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

      // if our angular velocity is too fast, ignore vision updates
      if (Math.abs(getPigeon2( ).getAngularVelocityZWorld( ).getValue( ).in(DegreesPerSecond)) > 360)
      {
        doRejectUpdate = true;
      }

      if (mt2 == null || mt2.tagCount == 0) // If no tags are visible, reject the update
      {
        doRejectUpdate = true;
      }
      else if (mt2.avgTagDist > 3.2)        // Reject if average tag distance is greater than 5 meters away
      {
        doRejectUpdate = true;
      }

      // Process the vision update if it is valid and not rejected

      if (!doRejectUpdate)
      {
        final double kBase = 0.4;             // Standard deviation scaling factor for vision updates based on tag count
        final double kProportional = 2.0;

        // Update the network table with the vision pose estimate for this limelight
        double[ ] array =
        {
            mt2.pose.getX( ), mt2.pose.getY( ), mt2.pose.getRotation( ).getDegrees( )
        };
        poseArray.set(array);

        // Code used by some teams to scale std devs by distance (below) and used by several teams

        if (m_useLimelightEntry.get( ))
        {
          setVisionMeasurementStdDevs(VecBuilder.fill(                        //
              Math.pow(kBase, mt2.tagCount) * kProportional * mt2.avgTagDist, // Scale the X std dev by the tag count and the average distance
              Math.pow(kBase, mt2.tagCount) * kProportional * mt2.avgTagDist, // Scale the Y std dev by the tag count and the average distance
              Double.POSITIVE_INFINITY));                                     // Never use vision to update rotation
          super.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);         // Update the pose estimator with the vision measurement
        }
      }
    }

    return !doRejectUpdate;
  }

  /****************************************************************************
   * 
   * Resets swerve pose and limelight orientation
   * 
   * @param pose
   *          new pose
   */
  public void resetPoseAndLimelight(Pose2d pose)
  {
    resetPose(pose);
    LimelightHelpers.SetRobotOrientation(Constants.kLLFrontName, pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(Constants.kLLBackName, pose.getRotation( ).getDegrees( ), 0, 0, 0, 0, 0);
  }

  /****************************************************************************
   * 
   * This method will take in desired robot-relative chassis speeds,
   * generate a swerve setpoint, then set the target state for each module
   *
   * @param speeds
   *          The desired robot-relative speeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds)
  {
    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    m_previousSetpoint = m_setpointGenerator.generateSetpoint(  //
        m_previousSetpoint, // The previous setpoint
        speeds, // The desired target speeds
        0.02 // The loop time of the robot code, in seconds
    );

    setControl(new SwerveRequest.ApplyRobotSpeeds( ).withSpeeds(m_previousSetpoint.robotRelativeSpeeds( )));
    // setModuleStates(m_previousSetpoint.moduleStates( )); // FUTURE: swerve setpoint generator sample code
  }

  ////////////////////////////////////////////////////////////////////////////
  /////////////////////// COMMAND FACTORIES //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   *
   * Reset robot pose command factory.
   * This command will reset the robot pose to the pose in the network table entry "setPose".
   * 
   * @return Command to reset robot pose
   */
  private Command getResetPoseCommand( )
  {
    return this
        .runOnce(( ) -> resetPoseAndLimelight(new Pose2d(new Translation2d(m_setPoseEntry.get( )[0], m_setPoseEntry.get( )[1]),
            new Rotation2d(m_setPoseEntry.get( )[2])))) //
        .withName("ResetOdometry").ignoringDisable(true);
  }

  /****************************************************************************
   *
   * Log current module positions and calculate an average distance if requested
   * 
   * @param end
   *          If true, calculate the average distance traveled by the modules since the last call to
   * @return Command to log module positions and calculate average distance
   */
  public Command getModulePositionsCommand(boolean end)
  {
    return this.runOnce(( ) ->
    {
      if (!end)
      {
        for (int i = 0; i < 4; i++)
        {
          m_moduleDistances[i] = this.getState( ).ModulePositions[i].distanceMeters;
        }
      }
      else
      {
        double average = 0;
        for (int i = 0; i < 4; i++)
        {
          m_moduleDistances[i] = this.getState( ).ModulePositions[i].distanceMeters - m_moduleDistances[i];
          average += Math.abs(m_moduleDistances[i]);
        }
        average /= 4;
        DataLogManager.log(String.format("%s:  0: %.3f 1: %.3f 2: %.3f 3: %.3f average %.3f", this.getName( ),
            m_moduleDistances[0], m_moduleDistances[1], m_moduleDistances[2], m_moduleDistances[3], average));
      }
    }).ignoringDisable(true);
  }

  /****************************************************************************
   *
   * Pathplanner path following command factory.
   * Construct a PathPlanner path following command used in autonomous
   * 
   * @param ppPath
   *          PathPlannerPath to follow
   * @return Command to follow the specified PathPlannerPath
   */
  public Command getPPPathCommand(PathPlannerPath ppPath)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(ppPath).withName("SwerveFollowPath");
  }

  /****************************************************************************
   *
   * Pathplanner path following command factory.
   * Create auto-align to pose alignment command with path following
   * 
   * @return poseAlignFollowCommand
   *         path following command to align to a pose scoring position
   */
  public Command getPoseAlignPPFollowCommand( )
  {
    Pose2d currentPose = m_driveStatePose.get( );         // Get the current robot pose from odometry
    Pose2d goalPose = Vision.findGoalPose(currentPose);   // Find the goal pose using the nearest AprilTag

    // Create a list of waypoints from the current pose to the goal pose
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, goalPose);

    // Generate a new PathPlannerPath using the waypoints and path constraints
    PathPlannerPath path =
        new PathPlannerPath(waypoints, kPathFindConstraints, null, new GoalEndState(0.0, goalPose.getRotation( )));
    path.preventFlipping = true;

    // Return a command that logs the current and goal poses, and then follows the generated path
    return new SequentialCommandGroup( //
        new LogCommand("AlignPosePPFollow", String.format("current %s goal %s", currentPose, goalPose)), //
        AutoBuilder.followPath(path) //
    ).withName("AlignToPosePPFollow");
  }

  /****************************************************************************
   *
   * Pathplanner path finding command factory.
   * Create auto-align to pose alignment command with path finding
   * 
   * @return poseAlignFindCommand
   *         path finding command to align to a pose scoring position
   */
  public Command getAlignToPosePPFindCommand( )
  {
    Pose2d currentPose = m_driveStatePose.get( );       // Get the current robot pose from odometry
    Pose2d goalPose = Vision.findGoalPose(currentPose); // Find the goal pose using the nearest AprilTag

    // Return a command that logs the current and goal poses, and then finds a path to the goal pose
    return new SequentialCommandGroup( //
        new LogCommand("AlignPosePPFind", String.format("current %s goal %s", currentPose, goalPose)), //
        AutoBuilder.pathfindToPose(goalPose, kPathFindConstraints, 0.0) //
    ).withName("AlignToPosePPFind");
  }

  /****************************************************************************
   * 
   * Auto-aiming control using current robot pose from odometry
   * 
   * @param diffAngle
   *          difference between robot rotation and rotation to hub
   * @param maxAngularRate
   *          max angular rate to scale against
   * @return desired proportional angular velocity to rotate the chassis
   */
  public AngularVelocity aimPoseProportional(double diffAngle, AngularVelocity maxAngularRate)
  {
    double proportionalFactor = diffAngle * kAimingKp;

    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Auto-aiming control using current robot pose from odometry
   * 
   * @param distanceToHub
   *          distance to the hub in meters
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  public LinearVelocity rangePoseProportional(double distanceToHub, LinearVelocity maxSpeed)
  {
    double proportionalFactor = -(kOptimalDistance - distanceToHub) * kDrivingKp;

    return maxSpeed.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Command factory for auto-aligning to the hub using Limelight vision and odometry.
   * This command will calculate the difference in angle and distance to the hub, and then apply a
   * proportional control to drive the robot towards the hub while aligning its rotation.
   * Only aligns the front Limelight.
   * 
   * @return auto-align to hub command
   */
  public Command getAutoAlignToHubCommand( )
  {
    // Return a command that applies a swerve request based on the current robot pose and the hub position
    return this.applyRequest(( ) ->
    {
      // Sets hub center depending on whether alliance is red, blue, or missing
      Optional<Alliance> alliance = DriverStation.getAlliance( );
      m_hubCenter =
          (alliance.equals(Optional.of(DriverStation.Alliance.Red))) ? Constants.kHubCenterRed : Constants.kHubCenterBlue;

      Pose2d robotPose = m_driveStatePose.get( );

      // Subtracts current robot pose from hub pose
      Translation2d translation = m_hubCenter.minus(robotPose.getTranslation( ));

      Rotation2d hubrotation = translation.getAngle( );

      // Gets difference in current robot rotation and that of the hub
      double diffAngle = hubrotation.minus(robotPose.getRotation( )).getDegrees( );

      // Gets distance from current robot position and that of the hub
      double distanceToHub = m_hubCenter.getDistance(robotPose.getTranslation( ));

      m_aimPub.set(diffAngle);
      m_rangePub.set(distanceToHub);

      // Return a new SwerveRequest that applies the calculated velocities and rotational rate to the robot
      return new SwerveRequest.RobotCentric( )
          .withVelocityX(this.rangePoseProportional(distanceToHub, TunerConstants.kSpeedAt12Volts))       //
          .withVelocityY(0)                                                                  //
          .withRotationalRate(this.aimPoseProportional(diffAngle, RotationsPerSecond.of(1.0))     //
      );
    });
  }

}
