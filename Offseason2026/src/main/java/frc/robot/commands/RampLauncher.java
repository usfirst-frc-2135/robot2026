
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

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
