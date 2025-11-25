package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class ScoreCamberCoralCommand extends Command {
    private final CarriageSubsystem carriageSubsystem;
    private Dimensionless speed;
    private final CamberDirection direction;

    public ScoreCamberCoralCommand(CarriageSubsystem carriageSubsystem, CamberDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = carriageSubsystem.getDefaultSpeed();
        this.direction = direction;
    }

    public ScoreCamberCoralCommand(CarriageSubsystem carriageSubsystem, Dimensionless speed, CamberDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = speed;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        if (direction == CamberDirection.LEFT) {
            carriageSubsystem.moveRollersLeft(speed);
        } else {
            carriageSubsystem.moveRollersRight(speed);
        }
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
