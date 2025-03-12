package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCommand extends SequentialCommandGroup {

    public ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, String direction, int level) {
        addCommands(
            new ParallelCommandGroup(
                new ElevatorMoveCommand(elevator, level),
                new BeamAdjustment(drivetrain, direction)
            ),
            new CoralOuttakeCommand(rollers, level, direction),
            new ElevatorMoveCommand(elevator, 1)
        );
    }
}