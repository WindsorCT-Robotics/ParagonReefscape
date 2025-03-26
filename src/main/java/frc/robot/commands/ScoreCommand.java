package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NotificationsSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCommand extends SequentialCommandGroup {

    public ScoreCommand(NotificationsSubsystem notification, CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, String direction, int level) {
        addCommands(
            new ParallelCommandGroup(
                new ElevatorMoveCommand(elevator, level),
                new BeamAdjustment(drivetrain, direction, 1)
            ),
            new CoralOuttakeCommand(rollers, level, direction)
        );
    }
}