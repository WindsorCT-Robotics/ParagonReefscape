package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RightBeamAdjustment extends Command {

    private final CommandSwerveDrivetrain drivetrain;

    private RobotCentric robotCentric;

    public RightBeamAdjustment(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityY(-0.8));
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