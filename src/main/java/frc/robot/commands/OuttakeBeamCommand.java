package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class OuttakeBeamCommand extends Command {
    private final CarriageSubsystem rollers;

    public OuttakeBeamCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        rollers.moveIntakeRollers();
        rollers.moveOuttakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return !rollers.isBeamBroken();
    }
}