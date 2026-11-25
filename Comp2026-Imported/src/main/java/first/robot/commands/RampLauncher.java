
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.Constants.KKConsts;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;

/**
 * Command to start launcher wheels to score
 */
public class RampLauncher extends SequentialCommandGroup
{
  /**
   * Group command to start the launcher wheels
   * 
   * @param launcher
   *          launcher subsystem
   * @param kicker
   *          kicker subsystem
   */
  public RampLauncher(Launcher launcher, Kicker kicker)
  {
    setName("RampLauncher");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Stop Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),
    
        new LogCommand(getName(), "Start up Launcher rollers "),
        launcher.getLauncherScoreCommand()
    
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
