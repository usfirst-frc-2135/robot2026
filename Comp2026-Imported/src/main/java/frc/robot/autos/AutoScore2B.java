
package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import frc.robot.commands.AcquireFuel;
import frc.robot.commands.LaunchFuel;
import frc.robot.commands.LogCommand;
import frc.robot.commands.StopIntaking;
import frc.robot.commands.StopLaunching;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore2B extends SequentialCommandGroup
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
  public AutoScore2B(List<PathPlannerPath> ppAuto, Drivetrain drivetrain, Intake intake, Hopper hopper, Kicker kicker,
      Launcher launcher)
  {
    setName("AutoScore2B");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel, score once, acquire fuel again and score"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
              new WaitCommand(0.15),
              drivetrain.getPPPathCommand(ppAuto.get(0))
            ),
            new AcquireFuel(intake, hopper)
          ),

        new LogCommand(getName(),"stop intaking and prime the launcher"),
        new ParallelCommandGroup(
            launcher.getLauncherScoreCommand(),
            new StopIntaking(intake, hopper)
          ),

        new LogCommand(getName(),"Launch fuel"),
        new LaunchFuel(intake, hopper, kicker, launcher).withTimeout(7.0),
        new StopLaunching(intake, hopper, kicker, launcher),

        new LogCommand(getName(),"Drive return path while intaking "),
        new ParallelCommandGroup(
            drivetrain.getPPPathCommand(ppAuto.get(1)),
            new AcquireFuel(intake,hopper)
          ),
        new ParallelCommandGroup(
            launcher.getLauncherScoreCommand(),
            new StopIntaking(intake, hopper)
          ),
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
