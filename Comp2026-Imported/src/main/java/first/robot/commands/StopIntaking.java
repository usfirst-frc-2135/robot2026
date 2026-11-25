
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.Constants.HPConsts;
import first.robot.Constants.INConsts;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;

/**
 * Command to stop intaking fuel from floor
 */
public class StopIntaking extends SequentialCommandGroup
{
  /**
   * Group command to stop using the intake to acquire a fuel
   * 
   * @param intake
   *          intake subsystem
   * @param hopper
   *          hopper subsystem
   */
  public StopIntaking(Intake intake, Hopper hopper)
  {
    setName("StopIntaking");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Stop rollers and Hold the intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getProtectedAngle),

        new LogCommand(getName(), "Stop Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.STOP)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
