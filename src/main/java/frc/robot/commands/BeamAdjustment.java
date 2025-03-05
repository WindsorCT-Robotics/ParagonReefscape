package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class BeamAdjustment extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final String direction;

    public BeamAdjustment(CommandSwerveDrivetrain drivetrain, String direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        double velocityY;
        if (this.direction.equals("left")) {
            velocityY = 0.6;
        } else {
            velocityY = -0.6;
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