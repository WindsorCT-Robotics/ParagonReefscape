package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TranslateRelativeCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final RobotCentric robotCentric;

    public TranslateRelativeCommand(
        CommandSwerveDrivetrain drivetrain,
        RobotCentric robotCentric) {
        this.drivetrain = drivetrain;

        this.robotCentric = robotCentric;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        drivetrain.setControl(robotCentric);
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}