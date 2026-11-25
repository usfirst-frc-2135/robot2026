
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.Constants.HPConsts;
import first.robot.Constants.KKConsts;
import first.robot.Constants.INConsts.INRollerMode;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;

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
  public StopLaunching(Intake intake, Hopper hopper, Kicker kicker, Launcher launcher)
  {
    setName("StopLaunching");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Stop Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.STOP),

        new LogCommand(getName(), "Stop Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),

        new LogCommand(getName(), "Set Launcher to primed RPM"),
        launcher.getLauncherPrimedCommand(),

        new LogCommand(getName(), "Move Intake -45 degrees"),
        intake.getMoveToAngleCommand(INRollerMode.STOP, intake::getIndexingMedium)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
