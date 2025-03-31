package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;

public class RepositionCoralCommand extends Command {
    private final CarriageSubsystem rollers;
    private boolean beamBroke = false;
    private boolean stop = false;
    private double startTime = 0;

    public RepositionCoralCommand(CarriageSubsystem rollers) {
        this.rollers = rollers;
    }

    @Override
    public void initialize() {
        if (!rollers.isBeamBroken()) {
            stop = true;
        }
        beamBroke = false;
    }

    @Override
    public void execute() {
        if (rollers.isBeamBroken()) {
            if (!beamBroke) {
                rollers.manualMoveRollers(new Percent(-0.25));
            }
        } else {
            rollers.manualMoveRollers(new Percent(0.25));
            if (!beamBroke) {
                startTime = Timer.getFPGATimestamp();
            }
            beamBroke = true;
        }
    }

    @Override
    public void end(boolean interrupt) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        // System.out.println(elapsedTime);
        return beamBroke && rollers.isBeamBroken() && (elapsedTime > 0.05) || stop;
    }
}