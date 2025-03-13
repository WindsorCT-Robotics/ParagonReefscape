package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class RepositionCoralCommand extends Command {
    private final CarriageSubsystem rollers;
    private boolean beamBroke;
    private boolean stop;

    public RepositionCoralCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        if (!rollers.isBeamBroken()) {
            stop = true;
        }
        beamBroke = false;
        rollers.moveRollers(true);
    }

    @Override
    public void execute() {
        if (rollers.isBeamBroken()) {
            rollers.moveRollers(true);
        } else {
            rollers.moveRollers(false);
            beamBroke = true;
        }
    }

    @Override
    public void end(boolean interrupt) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return beamBroke && rollers.isBeamBroken() || stop;
    }
}
