
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.subsystems.Climber;

/**
 * Command to Climb the tower
 */
public class ClimbTower extends SequentialCommandGroup
{
  /**
   * Group command to use the climber to climb the tower
   * 
   * @param climber
   *          climber subsystem
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
