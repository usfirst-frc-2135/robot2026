
//
// Vision Class - handle limelight interface
//
package first.robot.lib;

import java.util.Optional;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.filter.MedianFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.system.DataLogManager;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

import com.limelightvision.IMUMode;
import com.limelightvision.LEDMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateConfig;
import com.pathplanner.lib.util.FlippingUtil;

import first.robot.Constants;
import first.robot.Constants.VIConsts;

/****************************************************************************
 * 
 * Vision class
 */
public class Vision
{
  // Constants
  private static final double kAimingKp            = 0.01;
  private static final double kDrivingKp           = 0.08;
  private static final double kTXOffset            = 0.0;
  private static final double kTYOffset            = 8.0;

  // Objects

  Limelight                   m_llFront            = new Limelight(Constants.kLLFrontName);
  Limelight                   m_llBack             = new Limelight(Constants.kLLBackName);

  /* What to publish over networktables for telemetry */
  private String              m_name               = "";

  // Declare module variables
  private MedianFilter        m_filteredTX         = new MedianFilter(5); // Filter to smooth out the TX value
  private MedianFilter        m_filteredTY         = new MedianFilter(5); // Filter to smooth out the TY value

  double                      untrusted            = PoseEstimateConfig.UNTRUSTED;

  // Start from the default so you do not have to set every value.
  PoseEstimateConfig          mt1Config            = PoseEstimateConfig.defaultMT1( )  //
      .withMinTagCount(1)                                       // At least one tag is required
      .withMaxSingleTagAmbiguity(0.7)                 // MT1 needs low-ambiguity perspectives
      .withMaxSingleTagDistance(3.0)              // Trust a single tag only within 3 m
      .withMaxAvgTagDistance(6.0)                   // With several tags, average distance under 6 m
      .withMinAvgTagArea(0.05)                                // Minimum average tag area           
      .withFieldBounds(16.541, 8.069)   // Reject estimates outside the field
      .withFieldBoundsMargin(0.5)                               // Allow 0.5 m past the field walls
      .withStdDevXY(0.5, 0.05, 2.0)         // 0.5 base, minimum 0.05, maximum 2.0
      .withStdDevTheta(untrusted, untrusted, untrusted)                       // Never fuse vision heading
      .withStdDevDistanceScaling(1.0, 0.0, 6.0) // Linear scaling
      .withStdDevTagCountDivision(0.5);                             // Divide by tag count

  PoseEstimateConfig          mt2Config            = PoseEstimateConfig.defaultMT2( ) //
      .withMinTagCount(1)   //
      .withMaxSingleTagAmbiguity(1.0)                   // MT2 handles ambiguous perspectives, accept all
      .withMaxSingleTagDistance(0.0)                // 0 disables this check
      .withMaxAvgTagDistance(8.0)                     // With several tags, average distance under 8 m            
      .withMinAvgTagArea(0.02)                                  // Minimum average tag area
      .withFieldBounds(16.541, 8.069)     // Reject estimates outside the field
      .withFieldBoundsMargin(0.5)                                 // Allow 0.5 m past the field walls
      .withStdDevXY(0.3, 0.0001, 2.0)         // 0.3 base, minimum 0.0001, maximum 2.0
      .withStdDevTheta(untrusted, untrusted, untrusted)                         // Never fuse vision heading  
      .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Square root of distance for MT2
      .withStdDevTagCountDivision(0.5);                               // Divide by tag count

  Pose3d                      cameraPoseRobotSpace = new Pose3d(0.30, 0.0, 0.20, new Rotation3d( ));
  Limelight                   camera               = new Limelight("limelight", cameraPoseRobotSpace)  //
      .withPoseEstimateConfig_MT1(mt1Config)    //
      .withPoseEstimateConfig_MT2(mt2Config)    //
      .withTelemetry(true);           // Keep accepted and rejected poses visible on dashboards
  boolean                     useMegaTag2          = true;

  // Each robot loop
  // PoseEstimateType type = PoseEstimateType.MT1_WPIBLUE;
  // if (useMegaTag2) {
  //   type = PoseEstimateType.MT2_WPIBLUE;
  //   // Use your robot pose heading here, not your raw IMU reading.
  //   Limelight.setSharedRobotOrientation(robotYawDegrees);
  // }

  // for (var estimate : camera.readAcceptedPoseEstimates(type)) {
  //   poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
  // }

  /****************************************************************************
   * 
   * Constructor
   */
  public Vision( )
  {
    setName("Vision");

    loadFieldAndDisplayPoses( );       // Identify the field and print useful poses

    initialize( );
  }

  /****************************************************************************
   * 
   * Getter and setter for managing the class name
   */
  private void setName(String name)
  {
    m_name = name;
  }

  public String getName( )
  {
    return m_name;
  }

  // Put methods for controlling this class here. Call these from Commands.

  /****************************************************************************
   * 
   * Initialize class during robot mode changes
   */
  public void initialize( )
  {
    DataLogManager.log(String.format("%s: Subsystem initialized!", getName( )));

    m_llFront.setLEDMode(LEDMode.FORCE_OFF);
    m_llBack.setLEDMode(LEDMode.FORCE_OFF);

    setCPUThrottleLevel(true);
    // setIMUModeExternalSeed( );

    if (MatchState.getAlliance( ).equals(Optional.of(Alliance.RED)))
    {
      setPriorityId(10, "RED");
    }
    else if (MatchState.getAlliance( ).equals(Optional.of(Alliance.BLUE)))
    {
      setPriorityId(26, "BLUE");
    }
    else
    {
      DataLogManager.log(String.format("%s: Driver station alliance color NOT SET!", getName( )));
    }

    // TODO: uncomment and test in cameras
    // LimelightHelpers.setCameraPose_RobotSpace(Constants.kLLFrontName, Constants.kLLFront_Forward, Constants.kLLFront_Side,
    //     Constants.kLLFront_Up, Constants.kLLFront_Pitch, Constants.kLLFront_Roll, Constants.kLLFront_Yaw);
    // LimelightHelpers.setCameraPose_RobotSpace(Constants.kLLBackName, Constants.kLLBack_Forward, Constants.kLLBack_Side,
    //     Constants.kLLBack_Up, Constants.kLLBack_Pitch, Constants.kLLBack_Roll, Constants.kLLBack_Yaw);
    // LimelightHelpers.setCropWindow(Constants.kLLFrontName, -1.0, 1.0, -1.0, 1.0);  // (left, right, top, bottom) crop window limits for target detection (range 0 to 1, where 0 is left or top of image, and 1 is right or bottom of image)
    // LimelightHelpers.setCropWindow(Constants.kLLBackName, -1.0, 1.0, -0.35, 1.0);  // (left, right, top, bottom) crop window limits for target detection (range 0 to 1, where 0 is left or top of image, and 1 is right or bottom of image)
    // LimelightHelpers.setFiducial3DOffset(Constants.kLLFrontName, 0.0, 0.0, 0.0);   // 3D point of interest offset from fiducial center (positive forward, right, up)
    // LimelightHelpers.setFiducial3DOffset(Constants.kLLBackName, 0.0, 0.0, 0.0);    // 3D point of interest offset from fiducial center (positive forward, right, up)

  }

  /****************************************************************************
   * 
   * Run vision during auto and teleop
   */
  public void run( )
  {
    DataLogManager.log(String.format("%s: Subsystem running!", getName( )));

    setCPUThrottleLevel(false);
    // setIMUModeAssistExternal( );
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity. Only aligns the front Limelight.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   * @return desired proportional angular velocity to rotate the chassis
   */
  public AngularVelocity aimProportional(AngularVelocity maxAngularRate)
  {
    double sampledTX = m_llFront.hasTarget( ) ? (m_llFront.getTXDegrees( ) + kTXOffset) : 0.0;

    double appliedTX = m_filteredTX.calculate(sampledTX);
    double proportionalFactor = -appliedTX * kAimingKp;

    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity. Only aligns the front Limelight.
   * 
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  public LinearVelocity rangeProportional(LinearVelocity maxSpeed)
  {
    double sampledTY = m_llFront.hasTarget( ) ? (m_llFront.getTYDegrees( ) + kTYOffset) : 0.0;

    double appliedTY = m_filteredTY.calculate(sampledTY);
    double proportionalFactor = -appliedTY * kDrivingKp;

    return maxSpeed.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Set priorityid and display alliance color
   * 
   * @param throttle
   *          Defaults to 0. Your Limelgiht will process one frame
   *          after skipping <throttle> frames.
   */
  public void setCPUThrottleLevel(boolean throttle)
  {
    DataLogManager.log(String.format("%s: Set Throttle level to %s", getName( ), throttle));
    m_llFront.setThrottle(throttle ? 100 : 0);
    m_llBack.setThrottle(throttle ? 100 : 0);
  }

  /****************************************************************************
   * 
   * Set IMU mode to passed parameter
   * 
   */
  private void setIMUModes(IMUMode mode)
  {
    DataLogManager.log(String.format("%s: Set IMU Mode to %d (%s)", getName( ), mode.toString( ), mode));
    m_llFront.setIMUMode(mode);
    m_llBack.setIMUMode(mode);
  }

  /****************************************************************************
   * 
   * Set IMU mode to EXTERNAL_SEED mode (load the LL4 internal IMU from robot IMU)
   * 
   */
  public void setIMUModeExternalSeed( )
  {
    setIMUModes(IMUMode.EXTERNAL);
  }

  /****************************************************************************
   * 
   * Set IMU mode to INTERNAL mode (use the LL4 internal IMU)
   * 
   */
  public void setIMUModeInternal( )
  {
    setIMUModes(IMUMode.INTERNAL);
  }

  /****************************************************************************
   * 
   * Set IMU mode to MT1 ASSIST mode (use the LL4 internal IMU with MT1 updates)
   * 
   */
  public void setIMUModeAssistMT1( )
  {
    setIMUModes(IMUMode.INTERNAL_MT1_ASSIST);
  }

  /****************************************************************************
   * 
   * Set IMU mode to External ASSIST mode (use the LL4 internal IMU with External updates)
   * 
   */
  public void setIMUModeAssistExternal( )
  {
    setIMUModes(IMUMode.INTERNAL_EXTERNAL_ASSIST);
  }

  /****************************************************************************
   * 
   * Set priorityid and display alliance color
   * 
   * @param id
   *          aprilTag ID to set as priority
   * @param alliance
   *          alliance color string selected
   */
  private void setPriorityId(int id, String alliance)
  {
    DataLogManager.log(String.format("%s: Set AprilTag priority id %d (%s)", getName( ), id, alliance));
    m_llFront.setPriorityTagIDOverride(id);
  }

  // ////////////////////////////////////////////////////////////////////////// 
  // ///////////////////// AUTO-ALIGN HELPERS /////////////////////////////////
  // //////////////////////////////////////////////////////////////////////////

  /****************************************************************************
   * 
   * Print out field layout, tag ID poses, and scoring poses
   */
  private void loadFieldAndDisplayPoses( )
  {
    // Identify the field and load it (any reference loads it)
    DataLogManager.log(String.format("Field: %s width %.2f length %.2f", VIConsts.kGameField, VIConsts.kATField.getFieldWidth( ),
        VIConsts.kATField.getFieldLength( )));

    // DataLogManager.log(String.format("-----"));
    // for (int i = 1; i <= 32; i++)
    // {
    //   DataLogManager.log(String.format("Field: ID %2d %s", i, VIConsts.kATField.getTagPose(i)));
    // }
    // DataLogManager.log(String.format("-----"));
  }

  /****************************************************************************
   * 
   * * Find Goal Pose for a given current pose
   * NOTE: Uses a well-known AprilTag ID (Rebuilt) until updated for a new game.
   * 
   * 1) Get the closest AprilTag
   * 5) Return the flipped goal pose based on red or blue alliance
   * 
   * @return goalPose
   *         goal pose for the current pose passed in
   */
  public static Pose2d findGoalPose(Pose2d currentPose)
  {
    Pose2d goalPose;

    Pose2d atPose = VIConsts.kATField.getTagPose(16).get( ).toPose2d( );
    Transform2d transform = new Transform2d(0, 0, Rotation2d.k180deg);
    goalPose = atPose.transformBy(transform);

    if (MatchState.getAlliance( ).orElse(Alliance.BLUE) == Alliance.RED)
    {
      goalPose = FlippingUtil.flipFieldPose(goalPose);
    }

    DataLogManager.log(String.format("Vision: goal pose %s", goalPose));
    return goalPose;
  }

}
