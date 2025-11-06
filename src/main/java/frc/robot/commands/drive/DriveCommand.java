package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends RunCommand {
    public DriveCommand(CommandSwerveDrivetrain drivetrain, Supplier<SwerveRequest> requestSupplier) {
        super(() -> drivetrain.setControl(requestSupplier.get()), drivetrain);
    }
}
