
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class AutoScore1A extends SequentialCommandGroup
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
  public AutoScore1A(List<PathPlannerPath> ppAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Hopper hopper,
      Kicker kicker, Launcher launcher)
  {
    setName("AutoScore");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel and score once"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppAuto.get(0)),
          new AcquireFuel(intake, hopper)
        ), 
        new LogCommand(getName(),"Drive return path while stop intaking and prime the launcher"),
        new ParallelCommandGroup(
          drivetrain.getPathCommand(ppAuto.get(1)),
          launcher.getLauncherScoreCommand( ),
          new StopIntaking(intake, hopper)
        ), 
        new WaitCommand(0.250),
        new LogCommand(getName(),"Launch fuel"),
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
