package frc.robot.commands.drive.timeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToSensor extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final HorizontalDirection direction;

    public AlignToSensor(CommandSwerveDrivetrain drivetrain, HorizontalDirection direction) {
        this.drivetrain = drivetrain;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        switch (direction) {
            case LEFT:
                
                break;
            case RIGHT:
                
                break;
        }
    }

    @Override
    public void execute() {
        switch (direction) {
            case LEFT:
                
                break;
            case RIGHT:
                
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        
    }
}
