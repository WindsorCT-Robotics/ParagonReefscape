package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCommand extends Command {
    private final CarriageSubsystem rollers;
    private final double level;
    private final String direction;

    public CoralOuttakeCommand(CarriageSubsystem rollers, double level, String direction) {
        this.rollers = rollers;
        this.level = level;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (level == 1) {
            if (direction.equalsIgnoreCase("right")) {
                rollers.moveRollersRight();
            } else {
                rollers.moveRollersLeft();
            }
        } else {
            rollers.moveRollers(false);
        }
    }

    @Override
    public void execute() {

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