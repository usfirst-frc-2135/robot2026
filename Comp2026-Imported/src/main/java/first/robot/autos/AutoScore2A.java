
package first.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import first.robot.commands.AcquireFuel;
import first.robot.commands.LaunchFuel;
import first.robot.commands.LogCommand;
import first.robot.commands.StopIntaking;
import first.robot.commands.StopLaunching;
import first.robot.subsystems.Drivetrain;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Launcher;

import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore2A extends SequentialCommandGroup
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
  public AutoScore2A(List<PathPlannerPath> ppAuto, Drivetrain drivetrain, Intake intake, Hopper hopper, Kicker kicker,
      Launcher launcher)
  {
    setName("AutoScore2A");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel and score once then acquire fuel again in tighter path then launch"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
              new WaitCommand(0.15),
              drivetrain.getPPPathCommand(ppAuto.get(0))
            ),
            new AcquireFuel(intake, hopper)
          ),

        new LogCommand(getName(),"Drive return path while stop intaking and prime the launcher"),
        new ParallelCommandGroup(
            drivetrain.getPPPathCommand(ppAuto.get(1)),
            launcher.getLauncherScoreCommand(),
            new StopIntaking(intake, hopper)
          ),

        new WaitCommand(0.150),

        new LogCommand(getName(),"Launch fuel"),
        new LaunchFuel(intake, hopper, kicker, launcher).withTimeout(7.0),
        new StopLaunching(intake, hopper, kicker, launcher),

        new LogCommand(getName(),"Drive third path while acquiring"),
        new ParallelCommandGroup(
            drivetrain.getPPPathCommand(ppAuto.get(2)),
            new AcquireFuel(intake,hopper)
          ),
        launcher.getLauncherScoreCommand(),
        new StopIntaking(intake, hopper),
        
        new WaitCommand(0.150),
        
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
