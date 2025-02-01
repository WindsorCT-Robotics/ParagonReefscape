package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCoralCommand extends SequentialCommandGroup{
    public IntakeCoralCommand(CarriageSubsystem intake, CarriageSubsystem outtake) {
        addCommands(
            new IntakeRollersCommand(intake), new OuttakeRollersCommand(outtake),
            new ParallelDeadlineGroup(
                new IntakeRollersBeamCommand(intake),
                new OuttakeRollersIntakeBeamCommand(outtake, intake)
            )
        );
    }
}