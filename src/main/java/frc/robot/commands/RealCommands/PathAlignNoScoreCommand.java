package frc.robot.commands.RealCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Carriage.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathAlignNoScoreCommand extends ParallelCommandGroup {

    public PathAlignNoScoreCommand(CarriageSubsystem rollers, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction) {
        addCommands(
            new SequentialCommandGroup(
                drivetrain.pathToAlign(limelight, false, direction),
                new BeamAdjustment(drivetrain, direction)
            )
        );
    }
}