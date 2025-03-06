package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class PathCommand extends SequentialCommandGroup {
    public PathCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, boolean isCoralStation, String direction) {
        addCommands(
            path(drivetrain, limelight, isCoralStation, direction)
        );
        addRequirements(drivetrain);
    }
    
    public Command path(CommandSwerveDrivetrain drivetrain, Limelight limelight, boolean isCoralStation, String direction) {
        return drivetrain.pathToAlign(limelight, isCoralStation, direction);
    }
}
