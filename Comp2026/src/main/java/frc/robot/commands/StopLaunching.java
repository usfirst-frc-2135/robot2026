
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;

/**
 * Command to stop launching fuel
 */
public class StopLaunching extends SequentialCommandGroup
{
  /**
   * Group command to stop using the launcher
   * 
   * @param hopper
   *          hopper subsystem
   * @param launcher
   *          launcher subsystem
   * @param kicker
   *          kicker subsystem
   * @param intake
   *          intake subsystem
   */
  public StopLaunching(Hopper hopper, Kicker kicker, Launcher launcher, Intake intake)
  {
    setName("StopLaunching");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Stop Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.STOP),

        new LogCommand(getName(), "Stop Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),

        new LogCommand(getName(), "Stop Launcher rollers "),
        launcher.getLauncherPrimedCommand(),

        new LogCommand(getName(), "Move Intake -45 degrees"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getHalfwayAngle)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
