
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.KKConsts;
import frc.robot.Constants.SHConsts;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

/**
 * Command to start launcher wheels to score
 */
public class RampShooter extends SequentialCommandGroup
{
    /**
     * Group command to start the launcher wheels
     * 
     * @param shooter
     *            shooter subsystem
     * @param kicker
     *            kicker subsystem
     */
    public RampShooter(Shooter shooter, Kicker kicker)
    {
        setName("RampShooter");

        addCommands(
                // Add Commands here:

                // @formatter:off
            new LogCommand(getName(), "Stop Kicker Rollers"), 
            kicker.getRollerModeCommand(KKConsts.KKRollerMode.STOP),
        
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
