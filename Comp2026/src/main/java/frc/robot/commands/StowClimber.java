
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

/**
 * Command to stow the climber
 */
public class StowClimber extends SequentialCommandGroup
{
  /**
   * Group command to put the climber in stowed position for gameplay
   * 
   * @param climber
   *          climber subsystem
   */
  public StowClimber(Climber climber)
  {
    setName("StowClimber");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Bring Climber back down"), 
        climber.getMoveToPositionCommand(climber::getClimberStowed)
       
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
