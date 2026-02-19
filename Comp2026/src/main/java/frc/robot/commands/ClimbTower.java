
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Command to acquire a fuel from floor
 */
public class ClimbTower extends SequentialCommandGroup
{
    /**
     * Group command to use the intake to acquire a fuel from the floor
     * 
     * @param climber
     *            climber subsystem
     */
    public ClimbTower(Climber climber)
    {
        setName("ClimbTower");

        addCommands(
                // Add Commands here: 

                // @formatter:off

        new LogCommand(getName(), "Start Kicker Rollers"), 
        climber.getMoveToPositionCommand(climber::getClimberClimbed)
       
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
