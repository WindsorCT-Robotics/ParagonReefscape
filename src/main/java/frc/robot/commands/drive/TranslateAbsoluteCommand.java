package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TranslateAbsoluteCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final FieldCentric fieldCentric;

    public TranslateAbsoluteCommand(
        CommandSwerveDrivetrain drivetrain,
        FieldCentric fieldCentric) {
        this.drivetrain = drivetrain;
        

        this.fieldCentric = fieldCentric;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        drivetrain.setControl(fieldCentric);
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