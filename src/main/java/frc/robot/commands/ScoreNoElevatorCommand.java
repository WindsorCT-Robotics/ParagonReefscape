package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.NotificationsSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreNoElevatorCommand extends SequentialCommandGroup {

    public ScoreNoElevatorCommand(NotificationsSubsystem notification, CarriageSubsystem rollers, CommandSwerveDrivetrain drivetrain, String direction) {
        addCommands(
            new BeamAdjustment(drivetrain, direction, 1),
            new CoralOuttakeCommand(rollers, 1, direction)
        );
    }
}