package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

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
    // Change servo settings to work with this linear actuator per the docs linked on this web page:
    // https://andymark.com/products/linear-servo-actuators?_pos=1&_sid=800c6cc56&_ss=r
    // Max PWM pulse 2.0 ms, min 1.0 ms, with reasonable middle ranges
    m_actuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    AddDashboardWidgets( ); // Add dashboard widgets for commands
  }

}
