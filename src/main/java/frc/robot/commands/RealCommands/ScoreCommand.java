package frc.robot.commands.RealCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Carriage.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreCommand extends SequentialCommandGroup {

    public ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, String direction, double level) {
        addCommands(
            new ParallelCommandGroup(
                new ElevatorMoveCommand(elevator, level),
                new BeamAdjustment(drivetrain, direction)
            ),
            new CoralOuttakeCommand(rollers, level, direction)
        );
    }
}