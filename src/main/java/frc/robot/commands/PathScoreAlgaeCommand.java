package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathScoreAlgaeCommand extends SequentialCommandGroup {

    public PathScoreAlgaeCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, int level) {
        if (direction.equals("left")) {
            addCommands(
                new ParallelCommandGroup(
                    new RepositionCoralCommand(rollers),
                    new SequentialCommandGroup(
                        drivetrain.pathToAlign(limelight, false, direction),
                        new BeamAdjustment(drivetrain, direction, 1)
                    ),
                    new ElevatorMoveCommand(elevator, level)
                ),
                new CoralOuttakeCommand(rollers, level, direction),
                new BeamAdjustment(drivetrain, "right", 2)
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new RepositionCoralCommand(rollers),
                    new SequentialCommandGroup(
                        drivetrain.pathToAlign(limelight, false, direction),
                        new BeamAdjustment(drivetrain, direction, 1)
                    ),
                    new ElevatorMoveCommand(elevator, level)
                ),
                new CoralOuttakeCommand(rollers, level, direction),
                new BeamAdjustment(drivetrain, "left", 2)
            );
        }
    }
}