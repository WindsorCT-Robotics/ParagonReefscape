package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TranslateRelativeCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LinearVelocity MaxVelocity;
    private final AngularVelocity MaxAngularRate;
    private RobotCentric robotCentric;
    private CommandXboxController controller;

    public TranslateRelativeCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.MaxVelocity = drivetrain.getMaxVelocity();
        this.MaxAngularRate = drivetrain.getMaxAngularRate();
        this.controller = controller;
    }

    @Override
    public void initialize() {
        new DriveCommand(drivetrain, () -> driveRequest(controller)).schedule();
    }

    private RobotCentric driveRequest(CommandXboxController controller) {
        return robotCentric
            .withVelocityX(-controller.getLeftY() * Math.abs(controller.getLeftY()) * MaxVelocity.in(MetersPerSecond)) // Drive forward with negative Y (forward)
            .withVelocityY(-controller.getLeftX() * Math.abs(controller.getLeftX()) * MaxVelocity.in(MetersPerSecond)) // Drive left with negative X (left)
            .withRotationalRate(-controller.getRightX() * Math.abs(controller.getRightX()) * MaxAngularRate.in(RadiansPerSecond));
    }
}
