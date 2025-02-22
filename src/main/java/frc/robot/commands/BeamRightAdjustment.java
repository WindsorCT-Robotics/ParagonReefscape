package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class BeamRightAdjustment extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    public BeamRightAdjustment(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(-0.6));
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
        return drivetrain.getRTOFBeam();
    }
}