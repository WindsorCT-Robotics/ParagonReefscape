package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SetOrientationCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private double direction;

    public SetOrientationCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    @Override
    public void initialize() {
        System.out.println(drivetrain.getState().Pose.getY());
    }

    @Override
    public void execute() {
        System.out.println(direction);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (drivetrain.getState().Pose.getY() >= 4.026) {
                direction = 306;
            } else {
                direction = 54;
            }
        } else {
            if (drivetrain.getState().Pose.getY() >= 4.026) {
                direction = 234;
            } else {
                direction = 126;
            }
        }
        // drivetrain.applyRequest(() -> new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(Rotation2d.fromDegrees(direction)).withHeadingPID(30, 0, 2));
    }


    public double getDirection() {
        initialize();
        execute();
        return direction;
    }
}
