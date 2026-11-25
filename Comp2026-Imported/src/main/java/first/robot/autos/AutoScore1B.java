
package first.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import first.robot.commands.AcquireFuel;
import first.robot.commands.LaunchFuel;
import first.robot.commands.LogCommand;
import first.robot.commands.StopIntaking;
import first.robot.subsystems.Drivetrain;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;

import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;

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
  public AutoScore1B(List<PathPlannerPath> ppAuto, Drivetrain drivetrain, Intake intake, Hopper hopper, Kicker kicker,
      Launcher launcher)
  {
    setName("AutoScore");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel with NZ path and score"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup( 
          drivetrain.getPPPathCommand(ppAuto.get(0)),
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
