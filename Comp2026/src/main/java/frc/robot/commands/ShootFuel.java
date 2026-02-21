
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Command to launch fuel
 */
public class ShootFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the launcher to shoot a fuel
   * 
   * @param hopper
   *          hopper subsystem
   * @param launcher
   *          Launcher subsystem
   * @param kicker
   *          kicker subsystem
   */
  public ShootFuel(Hopper hopper, Kicker kicker, Launcher launcher)
  {
    setName("ShootFuel");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Start Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE),

        new LogCommand(getName(), "Start Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.ACQUIRE),
      
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
