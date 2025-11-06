package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TranslateAbsoluteCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LinearVelocity MaxVelocity;
    private final AngularVelocity MaxAngularRate;
    private FieldCentric fieldCentric;
    private CommandXboxController controller;

    public TranslateAbsoluteCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.MaxVelocity = drivetrain.getMaxVelocity();
        this.MaxAngularRate = drivetrain.getMaxAngularRate();
        this.controller = controller;
    }

    @Override
    public void execute() {
        new DriveCommand(drivetrain, () -> centricDrive(controller));
    }

    private FieldCentric centricDrive(CommandXboxController controller) {
        return fieldCentric
            .withVelocityX(-controller.getLeftY() * Math.abs(controller.getLeftY()) * MaxVelocity.in(MetersPerSecond)) // Drive forward with negative Y (forward)
            .withVelocityY(-controller.getLeftX() * Math.abs(controller.getLeftX()) * MaxVelocity.in(MetersPerSecond)) // Drive left with negative X (left)
            .withRotationalRate(-controller.getRightX() * Math.abs(controller.getRightX()) * MaxAngularRate.in(RadiansPerSecond));
    }
}
