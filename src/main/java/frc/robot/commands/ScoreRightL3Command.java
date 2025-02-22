package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreRightL3Command extends SequentialCommandGroup {

    public ScoreRightL3Command(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new ParallelCommandGroup(
                new ElevatorExtendCommand(elevator),
                new BeamRightAdjustment(drivetrain)
            ),
            new BeamOuttakeCommand(rollers),
            new ElevatorRetractCommand(elevator)
        );
    }
}