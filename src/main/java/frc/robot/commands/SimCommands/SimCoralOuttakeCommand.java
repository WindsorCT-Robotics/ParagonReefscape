package frc.robot.commands.SimCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage.CarriageSubsystemSim;

public class SimCoralOuttakeCommand extends Command {
    private final CarriageSubsystemSim rollers;
    private final double level;
    private final double L1 = 0.5;
    private final double L2 = 0.81;
    private final double L3 = 1.21;

    public SimCoralOuttakeCommand(CarriageSubsystemSim rollers, double level) {
        this.rollers = rollers;
        this.level = level;
    }

    @Override
    public void initialize() {
        System.out.println("Scoring...");

        double height = 0;

        if (level == 1){
            height = L1;
        } else if (level == 2) {
            height = L2;
        } else if (level == 3) {
            height = L3;
        } else {
            height = 10;
        }
        rollers.scoreCoral(3, height);
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