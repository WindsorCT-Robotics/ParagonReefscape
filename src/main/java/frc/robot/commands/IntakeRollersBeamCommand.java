package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.Units.Percent;

public class IntakeRollersBeamCommand extends Command {
    private final CarriageSubsystem rollers;

    public IntakeRollersBeamCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
    rollers.moveIntakeRollers();
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopIntakeRollers();
    }

    @Override
    public boolean isFinished() {
        return !rollers.isBeamBroken();
    }
}