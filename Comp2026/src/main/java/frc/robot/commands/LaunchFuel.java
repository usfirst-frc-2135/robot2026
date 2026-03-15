
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.HPConsts;
import frc.robot.Constants.INConsts;
import frc.robot.Constants.KKConsts;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Launcher;

/**
 * Command to launch fuel
 */
public class LaunchFuel extends SequentialCommandGroup
{
  /**
   * Group command to use the launcher to launch a fuel
   * 
   * @param hopper
   *          hopper subsystem
   * @param launcher
   *          launcher subsystem
   * @param kicker
   *          kicker subsystem
   * @param intake
   *          intake subsystem
   */
  public LaunchFuel(Hopper hopper, Kicker kicker, Launcher launcher, Intake intake)
  {
    setName("LaunchFuel");

    addCommands(
        // Add Commands here: 

        // @formatter:off

        new LogCommand(getName(), "Start up Launcher rollers "),
        launcher.getLauncherScoreCommand(),

        new LogCommand(getName(), "Wait until launcher at full speeed "),
        new WaitUntilCommand(launcher::isAtRequestedRPM).withTimeout(1.2),

        new LogCommand(getName(), "Start Hopper Rollers"), 
        hopper.getRollerModeCommand(HPConsts.HPRollerMode.ACQUIRE),

        new LogCommand(getName(), "Start Kicker Rollers"), 
        kicker.getRollerModeCommand(KKConsts.KKRollerMode.ACQUIRE),

        new WaitUntilCommand(1.0),
        intake.getMoveToAngleCommand(INConsts.INRollerMode.STOP, intake::getStowedAngle)


    
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
}
