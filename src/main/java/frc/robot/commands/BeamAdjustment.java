package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class BeamAdjustment extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;
    private final double speed;

    public BeamAdjustment(CommandSwerveDrivetrain drivetrain, String direction, double speed) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        this.speed = speed;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        double velocityY;
        if (this.direction.equals("left")) {
            velocityY = speed;
        } else {
            velocityY = -speed;
        }
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(velocityY));
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        if (this.direction.equals("left")) {
            return drivetrain.getLTOFBeam();
        } else {
            return drivetrain.getRTOFBeam();
        }
    }
}