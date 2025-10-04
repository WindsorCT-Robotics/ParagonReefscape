package frc.robot.commands.RealCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreNoElevatorCommand extends SequentialCommandGroup {

    public ScoreNoElevatorCommand(CarriageSubsystem rollers, CommandSwerveDrivetrain drivetrain, String direction) {
        addCommands(
            new BeamAdjustment(drivetrain, direction),
            new CoralOuttakeCommand(rollers, 1, direction)
        );
    }
}