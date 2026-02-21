
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

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

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
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
