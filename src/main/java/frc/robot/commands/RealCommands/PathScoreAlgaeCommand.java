package frc.robot.commands.RealCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathScoreAlgaeCommand extends SequentialCommandGroup {

    public PathScoreAlgaeCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, Double level) {
        if (level == 2.0) {
            addCommands(
                new ParallelCommandGroup(
                    new RepositionCoralCommand(rollers),
                    drivetrain.pathToAlign(limelight, false, direction),
                    new ElevatorMoveCommand(elevator, 1.0)
                ),
                new BeamAdjustment(drivetrain, direction),
                new ElevatorMoveCommand(elevator, level),
                new CoralOuttakeCommand(rollers, level, direction)
            );
        } else {
            addCommands(
                new ParallelCommandGroup(
                    new RepositionCoralCommand(rollers),
                    drivetrain.pathToAlign(limelight, false, direction),
                    new ElevatorMoveCommand(elevator, 2.5)
                ),
                new BeamAdjustment(drivetrain, direction),
                new ElevatorMoveCommand(elevator, 3.0),
                new CoralOuttakeCommand(rollers, level, direction)
            );
        }
    }
}