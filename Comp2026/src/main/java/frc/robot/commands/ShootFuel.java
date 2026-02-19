
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.Constants.SHConsts;
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
     * @param launcher
     *            Launcher subsystem
     * @param kicker
     *            kicker subsystem
     *            * @param hopper
     *            hopper subsystem
     */
    public ShootFuel(Launcher launcher, Kicker kicker, Hopper hopper)
    {
        setName("ShootFuel");

        addCommands(
                // Add Commands here: 

                // @formatter:off

        new LogCommand(getName(), "Start Kicker Rollers"), 
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
