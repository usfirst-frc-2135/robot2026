
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore2 extends SequentialCommandGroup
{
    /**
     * Autonomous command to:
     * 1 - Leave the starting line
     * 
     * @param ppAuto
     *            list of auto paths to follow
     * @param drivetrain
     *            swerve drivetrain subsystem
     */
    public AutoScore2(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain)
    {
        setName("AutoScore2");

        addCommands(
                // Add Commands here:

                // @formatter:off

        new LogCommand(getName(), "Acquire fuel and score once"),
        drivetrain.getPathCommand(ppAuto.get(0)),
        drivetrain.getPathCommand(ppAuto.get(1)),
        drivetrain.getPathCommand(ppAuto.get(2)),
        drivetrain.getPathCommand(ppAuto.get(3)),
        drivetrain.getPathCommand(ppAuto.get(4))
        // @formatter:on
        );
    }

    @Override
    public boolean runsWhenDisabled( )
    {
        return false;
    }
}
