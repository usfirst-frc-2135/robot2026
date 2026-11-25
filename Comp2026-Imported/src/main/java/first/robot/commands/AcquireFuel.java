
package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;

import first.robot.Constants.HPConsts;
import first.robot.Constants.INConsts;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;

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

        new LogCommand(getName(), "Start rollers and Deploy intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getDeployedAngle),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.ACQUIRE, intake::getDeployedAngle),

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
