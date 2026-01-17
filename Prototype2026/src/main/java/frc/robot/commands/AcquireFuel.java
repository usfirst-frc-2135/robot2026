package frc.robot.commands;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.INConsts;
import frc.robot.subsystems.Intake;


public class AcquireFuel extends SequentialCommandGroup{
  public AcquireFuel(Intake intake)
  {
    setName("AcquireFuel");

    addCommands(
        // Add Commands here:

        // @formatter:off

        new LogCommand(getName(), "Start rollers & Deploy intake rotary"),
        intake.getMoveToPositionCommand(INConsts.INRollerMode.ACQUIRE),

        new LogCommand(getName(), "Wait for note"),
        new WaitUntilCommand(intake::isNoteDetected),

        new LogCommand(getName(), "Stop rollers & Retract intake rotary"),
        
        intake.getMoveToPositionCommand(INConsts.INRollerMode.STOP)
        
        // @formatter:on
    );
  }

  @Override
  public boolean runsWhenDisabled( )
  {
    return false;
  }
  
}
