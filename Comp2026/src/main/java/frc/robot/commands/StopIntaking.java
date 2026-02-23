
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

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

        new LogCommand(getName(), "Stop rollers & Hold the intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getCurrentAngle),

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
