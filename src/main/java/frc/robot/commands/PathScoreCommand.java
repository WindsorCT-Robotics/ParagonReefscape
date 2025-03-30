package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NotificationsSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathScoreCommand extends SequentialCommandGroup {

    public PathScoreCommand(NotificationsSubsystem notification, CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, Double level) {
        addCommands(
            new ParallelCommandGroup(
                new RepositionCoralCommand(rollers),
                new SequentialCommandGroup(
                    drivetrain.pathToAlign(limelight, false, direction),
                    new BeamAdjustment(drivetrain, direction)
                ),
                new ElevatorMoveCommand(elevator, level)
            ),
            new CoralOuttakeCommand(rollers, level, direction)
        );
    }
}
