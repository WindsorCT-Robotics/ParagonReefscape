package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleAllCommand extends ParallelCommandGroup {

    public RumbleAllCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController driver, CommandXboxController op, double rumblePercent) {
        addCommands(
            new RumbleCommand(drivetrain, limelight, driver, op, rumblePercent, 0),
            new RumbleCommand(drivetrain, limelight, driver, op, rumblePercent, 1)
        );
    }
}