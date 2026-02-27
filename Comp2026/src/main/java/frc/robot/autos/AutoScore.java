
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LogCommand;
import frc.robot.commands.StopIntaking;
import frc.robot.commands.AcquireFuel;
import frc.robot.commands.ExpelFuel;
import frc.robot.commands.LaunchFuel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Leave the starting line
   * 
   * @param ppAuto
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoScore(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Hopper hopper, Kicker kicker,
      Launcher launcher)
  {
    setName("AutoScore");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel and score once"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppAuto.get(0)),
          new AcquireFuel(intake, hopper)
        ), 
        new StopIntaking(intake, hopper),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppAuto.get(1)),
          launcher.getLauncherScoreCommand( )
        ), 
        new LaunchFuel(hopper, kicker, launcher)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
