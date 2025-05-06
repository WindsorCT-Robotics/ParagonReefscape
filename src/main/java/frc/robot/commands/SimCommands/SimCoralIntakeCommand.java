package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage.CarriageSubsystemSim;

public class SimCoralIntakeCommand extends Command {
    private final CarriageSubsystemSim rollers;

    public SimCoralIntakeCommand(CarriageSubsystemSim rollers) {
        this.rollers = rollers;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {
        System.out.println("Intaking...");
        rollers.moveRollers(true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        rollers.moveRollers(false);
    }

    @Override
    public boolean isFinished() {
        return rollers.isCoralInside();
    }
}