
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Command to Climb the tower
 */
public class ClimbTower extends SequentialCommandGroup
{
    /**
     * Group command to use the climber to climb the tower
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

        new LogCommand(getName(), "Climber "), 
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
