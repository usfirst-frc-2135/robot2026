
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.KKConsts;
import frc.robot.Constants.SHConsts;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

/**
 * Command to acquire a fuel from floor
 */
public class RampShooter extends SequentialCommandGroup
{
    /**
     * Group command to use the intake to acquire a fuel from the floor
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
        shooter.getShooterCommand(SHConsts.ShooterMode.SCORE)
    

        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
