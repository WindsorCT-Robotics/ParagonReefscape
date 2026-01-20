package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class ScoreCoralCommand extends Command {
    private final CarriageSubsystem carriageSubsystem;
    private Dimensionless speed;

    public ScoreCoralCommand(CarriageSubsystem carriageSubsystem) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = carriageSubsystem.getDefaultSpeed();
    }

    public ScoreCoralCommand(CarriageSubsystem carriageSubsystem, Dimensionless speed) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        carriageSubsystem.moveRollers(speed);
    }

    @Override
    public void end(boolean interrupted) {
        carriageSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return !carriageSubsystem.isBeamBroken();
    }
}