package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreNoElevatorCommand extends SequentialCommandGroup {

    public ScoreNoElevatorCommand(CarriageSubsystem rollers, CommandSwerveDrivetrain drivetrain, String direction) {
        addCommands(
            new BeamAdjustment(drivetrain, direction),
            new CoralOuttakeCommand(rollers)
        );
    }
}