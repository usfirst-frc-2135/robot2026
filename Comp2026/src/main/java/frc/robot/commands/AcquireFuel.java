
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;

/**
 * Command to acquire a note from floor
 */
public class AcquireFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to acquire a note from the floor
   * 
   * @param intake
   *          intake subsystem
   */
  public AcquireFuel(Intake intake)
  {
    setName("AcquireFuel");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.ACQUIRE, intake::getIntakeDeployed),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
       
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getIntakeRetracted)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
