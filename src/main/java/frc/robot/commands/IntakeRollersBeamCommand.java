package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class IntakeRollersBeamCommand extends Command {
    private final CarriageSubsystem intake;

    public IntakeRollersBeamCommand(CarriageSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        intake.moveIntakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return !intake.isBeamBroken();
    }
}