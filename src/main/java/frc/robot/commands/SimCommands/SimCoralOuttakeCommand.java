package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage.CarriageSubsystemSim;

public class SimCoralOuttakeCommand extends Command {
    private final CarriageSubsystemSim rollers;
    private final double level;

    public SimCoralOuttakeCommand(CarriageSubsystemSim rollers, double level) {
        this.rollers = rollers;
        this.level = level;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {
        System.out.println("Scoring...");
        rollers.scoreCoral(2, level);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}