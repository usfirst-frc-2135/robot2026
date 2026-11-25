
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitUntilCommand;

import first.robot.Constants.HPConsts;
import first.robot.Constants.KKConsts;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;

/**
 * Command to launch fuel
 */
public class LaunchFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the launcher to launch a fuel
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
  public LaunchFuel(Intake intake, Hopper hopper, Kicker kicker, Launcher launcher)
  {
    setName("LaunchFuel");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Start up Launcher rollers "),
        launcher.getLauncherScoreCommand(),

        new LogCommand(getName(), "Wait until launcher at full speed "),
        new WaitUntilCommand(launcher::isAtRequestedRPM).withTimeout(1.1),

        new LogCommand(getName(), "Start Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE),

        new LogCommand(getName(), "Start Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.ACQUIRE),

        new LogCommand(getName(), "Swing Intake Arm to index fuel"), 
        intake.getIndexingCommand()

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
