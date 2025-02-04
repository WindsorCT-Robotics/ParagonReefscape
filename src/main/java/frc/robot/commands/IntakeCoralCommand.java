package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCoralCommand extends SequentialCommandGroup{
    public IntakeCoralCommand(CarriageSubsystem rollers) {
        addCommands(
            new IntakeRollersCommand(rollers), new OuttakeRollersCommand(rollers),
            new ParallelDeadlineGroup(
                new OuttakeBeamCommand(rollers),
                new IntakeRollersCommand(rollers)
            )
        );
    }
}