
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireFuel;
import frc.robot.commands.LaunchFuel;
import frc.robot.commands.LogCommand;
import frc.robot.commands.StopIntaking;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore1B extends SequentialCommandGroup
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
  public AutoScore1B(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Hopper hopper,
      Kicker kicker, Launcher launcher)
  {
    setName("AutoScore");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel with NZ path and score"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup( 
          drivetrain.getPathCommand(ppAuto.get(0)),
          new AcquireFuel(intake, hopper)
        ), 
        new LogCommand(getName(),"Prime the launcher, stop intaking, launch fuel"),
        launcher.getLauncherScoreCommand( ),
        new StopIntaking(intake, hopper),
        new LaunchFuel(intake, hopper, kicker, launcher)

        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
