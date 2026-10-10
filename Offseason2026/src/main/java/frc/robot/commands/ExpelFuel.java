
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

/**
 * Command to expel a fuel to the floor
 */
public class ExpelFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the intake to expel a fuel to the floor
   * 
   * @param intake
   *          intake subsystem
   * @param hopper
   *          hopper subsystem
   * @param kicker
   *          kicker subsystem
   */
  public ExpelFuel(Intake intake, Hopper hopper, Kicker kicker)
  {
    setName("ExpelFuel");

    addCommands(
        // Add Commands here:

        // @formatter:off
        
        new LogCommand(getName(), "Stop rollers and Deploy intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getDeployedAngle),

        new LogCommand(getName(), "Expel rollers and Hold intake rotary in same position"),        
        intake.getMoveToAngleCommand(INConsts.INRollerMode.EXPEL, intake::getCurrentAngle),

        new LogCommand(getName(), "Run Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.EXPEL),

        new LogCommand(getName(), "Run Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.EXPEL)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
