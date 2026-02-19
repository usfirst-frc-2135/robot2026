
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

/**
 * Command to prepare climber to climb
 */
public class PrepareToClimb extends SequentialCommandGroup
{
    /**
     * Group command to prepare the climber to climb the tower
     * 
     * @param climber
     *            climber subsystem
     * @param intake
     *            intake subsystem
     */
    public PrepareToClimb(Climber climber, Intake intake)
    {
        setName("PrepareToClimb");

        addCommands(
                // Add Commands here: 

                // @formatter:off

            new LogCommand(getName(), "Retract intake"), 
            intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getIntakeRetracted),
            new LogCommand(getName(), "Extend Climber"), 
            climber.getMoveToPositionCommand(climber::getClimberFullyExtended)
       
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
