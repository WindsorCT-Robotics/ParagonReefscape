package frc.robot.commands.RealCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage.CarriageSubsystem;

public class CoralIntakeCommand extends Command {
    private final CarriageSubsystem rollers;

    public CoralIntakeCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        rollers.moveRollers(false);
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return rollers.isBeamBroken();
    }
}