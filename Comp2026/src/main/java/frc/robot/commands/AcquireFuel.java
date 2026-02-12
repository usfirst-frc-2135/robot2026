
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

/**
 * Command to acquire a fuel from floor
 */
public class AcquireFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to acquire a fuel from the floor
   * 
   * @param intake
   *          intake subsystem
   * @param hopper
   *          hopper subsystem
   */
  public AcquireFuel(Intake intake, Hopper hopper)
  {
    setName("AcquireFuel");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.ACQUIRE, intake::getIntakeDeployed),

        //new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        //intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getIntakeRetracted),

        new LogCommand(getName(), "Run Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
