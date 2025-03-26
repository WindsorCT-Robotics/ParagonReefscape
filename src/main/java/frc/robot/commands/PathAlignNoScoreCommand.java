package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathAlignNoScoreCommand extends ParallelCommandGroup {

    public PathAlignNoScoreCommand(CarriageSubsystem rollers, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction) {
        addCommands(
            new RepositionCoralCommand(rollers),
            new SequentialCommandGroup(
                drivetrain.pathToAlign(limelight, false, direction),
                new BeamAdjustment(drivetrain, direction, 1)
            )
        );
    }
}