package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Todo: This command won't work as intended due to missing velocity components in the SwerveRequest because the purpose is to be able to drive with a set angle.
public class SetAngleCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Angle> targetAngle;
    private final 

    public SetAngleCommand(CommandSwerveDrivetrain drivetrain, Supplier<Angle> targetAngle) {
        this.drivetrain = drivetrain;
        this.targetAngle = targetAngle;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setControl(turnRequest());
    }

    private SwerveRequest turnRequest() {
        return new SwerveRequest.FieldCentricFacingAngle()
                .withHeadingPID(drivetrain.getRotationPID().kP, drivetrain.getRotationPID().kI, drivetrain.getRotationPID().kD)
                .withTargetDirection(new Rotation2d(targetAngle.get().in(Degrees))); // Todo: Add .withVelocityX & .withVelocityY
    }

    @Override
    public void end(boolean interrupted) {
        // Todo: Determine what should happen when the command ends
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
