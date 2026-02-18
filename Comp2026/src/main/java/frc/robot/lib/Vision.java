
//
// Vision Class - handle limelight interface
//
package frc.robot.lib;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.VIConsts;

/****************************************************************************
 * 
 * Vision class
 */
public class Vision
{

  /** Camera stream mode parameter */
  private enum streamMode
  {
    STANDARD(0),    //
    PIPMAIN(1),     //
    PIPSECONDARY(2);

    @SuppressWarnings("unused")
    public final int value;

    private streamMode(int value)
    {
      this.value = value;
    }
  };

  /** Camera stream mode parameter */
  private enum imuMode
  {
    EXTERNAL(0),        // Use external IMU
    EXTERNAL_SEED(1),   // Use external IMU, seed internal
    INTERNAL(2),        // Use internal
    INTERNAL_MT1(3),    // Use internal with MT1 assisted convergence
    INTERNAL_ASSIST(4)  // Use internal IMU with external IMU assisted convergence
    ;

    public final int value;

    private imuMode(int value)
    {
      this.value = value;
    }
  };

  // Constants
  private static final double kAimingKp  = 0.01;
  private static final double kDrivingKp = 0.06;

  // Objects

  /* What to publish over networktables for telemetry */
  private String              m_name     = new String( );

  // Declare module variables
  @SuppressWarnings("unused")
  private streamMode          m_stream   = streamMode.STANDARD;

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

    LimelightHelpers.setLEDMode_ForceOff(Constants.kLLFrontName);         // These work on LL3 and lower (not LL4)
    LimelightHelpers.setLEDMode_ForceOff(Constants.kLLBackName);          // These work on LL3 and lower (not LL4)
    LimelightHelpers.setStreamMode_PiPSecondary(Constants.kLLFrontName);  // These work on LL3 and lower (not LL4)
    LimelightHelpers.setStreamMode_PiPSecondary(Constants.kLLBackName);   // These work on LL3 and lower (not LL4)

    SetCPUThrottleLevel(true);
    SetIMUModeExternalSeed( );
  }

  /****************************************************************************
   * 
   * Run vision during auto and teleop
   */
  public void run( )
  {
    DataLogManager.log(String.format("%s: Subsystem running!", getName( )));

    SetCPUThrottleLevel(false);
    SetIMUModeInternal( );
  }

  /****************************************************************************
   * 
   * Limelight auto-aiming control for rotational velocity. Only aligns the left Limelight.
   * 
   * @param maxAngularRate
   *          max angular rate to scale against
   * @return desired proportional angular velocity to rotate the chassis
   */
  public AngularVelocity aimProportional(AngularVelocity maxAngularRate)
  {
    double proportionalFactor = -LimelightHelpers.getTX(Constants.kLLFrontName) * kAimingKp;

    return maxAngularRate.times(proportionalFactor);
  }

  /****************************************************************************
   * 
   * Limelight auto-ranging control for distance velocity. Only aligns the left Limelight.
   * 
   * @param maxSpeed
   *          max speed to scale against
   * @return desired proportional linear velocity in chassis forward direction
   */
  public LinearVelocity rangeProportional(LinearVelocity maxSpeed)
  {
    double proportionalFactor = LimelightHelpers.getTY(Constants.kLLFrontName) * kDrivingKp;

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
  public void SetCPUThrottleLevel(boolean throttle)
  {
    DataLogManager.log(String.format("%s: Set Throttle level to %s", getName( ), throttle));
    LimelightHelpers.SetThrottle(Constants.kLLFrontName, throttle ? 100 : 0);
    LimelightHelpers.SetThrottle(Constants.kLLBackName, throttle ? 100 : 0);
  }

  /****************************************************************************
   * 
   * Set IMU mode to EXTERNAL_SEED mode (load the LL4 internal IMU from robot IMU)
   * 
   */
  public void SetIMUModeExternalSeed( )
  {
    final imuMode mode = imuMode.EXTERNAL_SEED;
    DataLogManager.log(String.format("%s: Set IMU Mode to %d (%s)", getName( ), mode.value, mode));
    // LimelightHelpers.SetIMUMode(Constants.kLLFrontName, mode.value);
    // LimelightHelpers.SetIMUMode(Constants.kLLBackName, mode.value);
  }

  /****************************************************************************
   * 
   * Set IMU mode to INTERNAL mode (use the LL4 internal IMU)
   * 
   */
  public void SetIMUModeInternal( )
  {
    final imuMode mode = imuMode.INTERNAL;
    DataLogManager.log(String.format("%s: Set IMU Mode to %d (%s)", getName( ), mode.value, mode));
    // LimelightHelpers.SetIMUMode(Constants.kLLFrontName, mode.value);
    // LimelightHelpers.SetIMUMode(Constants.kLLBackName, mode.value);
  }

  ////////////////////////////////////////////////////////////////////////////
  /////////////////////// AUTO-ALIGN HELPERS /////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////

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
   * TODO: use AprilTag ID until we have real poses
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

    if (DriverStation.getAlliance( ).orElse(Alliance.Blue) == Alliance.Red)
    {
      goalPose = FlippingUtil.flipFieldPose(goalPose);
    }

    DataLogManager.log(String.format("Vision: goal pose %s", goalPose));
    return goalPose;
  }

}
