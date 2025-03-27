package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathScoreAlgaeCommand extends SequentialCommandGroup {

    public PathScoreAlgaeCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, Double level) {
        if (level == 2.0) {
            addCommands(
                new ParallelCommandGroup(
                    // new RepositionCoralCommand(rollers),
                    drivetrain.pathToAlign(limelight, false, direction),
                    new ElevatorMoveCommand(elevator, 1.0)
                ),
                new BeamAdjustment(drivetrain, direction, 1),
                new ElevatorMoveCommand(elevator, level),
                new CoralOuttakeCommand(rollers, level, direction)
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    // new RepositionCoralCommand(rollers),
                    drivetrain.pathToAlign(limelight, false, direction),
                    new ElevatorMoveCommand(elevator, 2.5)
                ),
                new BeamAdjustment(drivetrain, direction, 1),
                new ElevatorMoveCommand(elevator, 3.0),
                new CoralOuttakeCommand(rollers, level, direction)
            );
        }
    }
}