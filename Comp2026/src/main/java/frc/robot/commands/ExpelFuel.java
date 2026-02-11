
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;

/**
 * Command to expel a fuel to the floor
 */
public class ExpelFuel extends SequentialCommandGroup
{
    /**
     * Group command to use the intake to expel a fuel to the floor
     * 
     * @param intake
     *            intake subsystem
     */
    public ExpelFuel(Intake intake)
    {
        setName("ExpelFuel");

        addCommands(
                // Add Commands here:

                // @formatter:off
        
        new LogCommand(getName(), "Stop rollers & Deploy intake rotary"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getIntakeDeployed),

        new LogCommand(getName(), "Expel rollers & Hold intake rotary in same position"),        
        intake.getMoveToAngleCommand(INConsts.INRollerMode.EXPEL, intake::getCurrentAngle),

        new LogCommand(getName(), "Wait for fuel to release"),  //TODO: is this needed?
        new WaitCommand(0.5),

        new LogCommand(getName(), "Stop rollers & Hold intake rotary in same position"),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getCurrentAngle)
        
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
