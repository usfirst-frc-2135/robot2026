
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.Constants.HPConsts;
import first.robot.Constants.INConsts;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;

/**
 * Command to bring the intake back
 */
public class RetractIntake extends SequentialCommandGroup
{
  /**
   * Group command to use retract intake back from the floor
   * 
   * @param intake
   *          intake subsystem
   * @param hopper
   *          hoper subsystem
   */
  public RetractIntake(Intake intake, Hopper hopper)
  {
    setName("RetractIntake");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Stop rollers and Retract intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getStowedAngle),

        new LogCommand(getName(), "Run Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.STOP)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
