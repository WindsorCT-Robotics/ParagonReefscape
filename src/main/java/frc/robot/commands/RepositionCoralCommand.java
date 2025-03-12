package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

    private final CarriageSubsystem rollers;
    private boolean beamBroke;

    public RepositionCoralCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        beamBroke = false;
        rollers.moveRollers(true);
    }

    @Override
    public void execute() {
        if (rollers.isBeamBroken()) {
            rollers.moveRollers(true);
        } else {
            rollers.moveRollers(false);
            if (rollers.isBeamBroken()) {
                beamBroke = true;
            }
        }
    }

    @Override
    public void end(boolean interrupt) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return beamBroke;
    }
}
