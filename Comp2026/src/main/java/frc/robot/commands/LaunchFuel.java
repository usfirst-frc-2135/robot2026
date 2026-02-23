
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Command to launch fuel
 */
public class LaunchFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the launcher to lauch a fuel
   * 
   * @param hopper
   *          hopper subsystem
   * @param launcher
   *          launcher subsystem
   * @param kicker
   *          kicker subsystem
   */
  public LaunchFuel(Hopper hopper, Kicker kicker, Launcher launcher)
  {
    setName("LaunchFuel");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Start up Launcher rollers "),
        launcher.getLauncherScoreCommand(),

        new LogCommand(getName(), "Wait until launcher at full speeed "),
        new WaitUntilCommand(launcher::isAtTargetRPM).withTimeout(2.0),

        new LogCommand(getName(), "Start Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE),

        new LogCommand(getName(), "Start Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.ACQUIRE)
    
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
