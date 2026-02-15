
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.Constants.SHConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

/**
 * Command to acquire a fuel from floor
 */
public class ShootFuel extends SequentialCommandGroup
{
    /**
     * Group command to use the intake to acquire a fuel from the floor
     * 
     * @param shooter
     *            shooter subsystem
     * @param kicker
     *            kicker subsystem
     *            * @param hopper
     *            hopper subsystem
     */
    public ShootFuel(Shooter shooter, Kicker kicker, Hopper hopper)
    {
        setName("ShootFuel");

        addCommands(
                // Add Commands here: 

                // @formatter:off

        new LogCommand(getName(), "Start Kicker Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE),

        new LogCommand(getName(), "Start Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.ACQUIRE),
      
        new LogCommand(getName(), "Start up Shooter rollers "),
        shooter.getShooterScoreCommand()
    

        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
