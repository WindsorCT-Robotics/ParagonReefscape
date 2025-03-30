package frc.robot.commands;

import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class MoveDrivetrainCommand extends Command {
    private final double speed;
    private final CommandSwerveDrivetrain drivetrain;
    private final double angle;
    private final boolean relative;
    private double xVel = 0;
    private double yVel = 0;

    public MoveDrivetrainCommand(CommandSwerveDrivetrain drivetrain, double speed, double angle, boolean relative) {
        this.speed = speed;
        this.drivetrain = drivetrain;
        this.angle = angle;
        this.relative = relative;
    }

    @Override
    public void initialize() {
        xVel = speed * Math.cos(Math.toRadians(angle));
        yVel = speed * Math.sin(Math.toRadians(angle));
    }

    @Override
    public void execute() {
        if (relative) {
            drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(xVel).withVelocityY(yVel));
        } else {
            drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(xVel).withVelocityY(yVel));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}