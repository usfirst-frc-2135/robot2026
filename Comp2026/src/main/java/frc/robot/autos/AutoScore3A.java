
package frc.robot.autos;

import java.util.List;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AcquireFuel;
import frc.robot.commands.LaunchFuel;
import frc.robot.commands.LogCommand;
import frc.robot.commands.StopIntaking;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Auto command that just leaves the starting zone
 */
public class AutoScore3A extends SequentialCommandGroup
{
  /**
   * Autonomous command to:
   * 1 - Leave the starting line
   * 
   * @param blineAuto
   *          list of auto paths to follow
   * @param drivetrain
   *          swerve drivetrain subsystem
   */
  public AutoScore3A(List<Path> blineAuto, CommandSwerveDrivetrain drivetrain, Intake intake, Hopper hopper, Kicker kicker,
      Launcher launcher)
  {
    setName("AutoScore2B");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Acquire fuel, score once, acquire fuel again and score"),
        new LogCommand(getName(),"Drive path while acquiring"),
        new ParallelCommandGroup(
          drivetrain.getPathBuilder( ).build(blineAuto.get(0)),
            new AcquireFuel(intake,hopper)
          ),
        new LogCommand(getName(),"stop intaking and prime the launcher"),
        new ParallelCommandGroup(
            launcher.getLauncherScoreCommand(),
            new StopIntaking(intake, hopper)
          ),
        new LogCommand(getName(),"Launch fuel"),
        new LaunchFuel(intake, hopper, kicker, launcher).withTimeout(3.5),

        new LogCommand(getName(),"Drive return path while intaking "),
        new ParallelCommandGroup(
            drivetrain.getPathBuilder( ).build(blineAuto.get(1)),
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
