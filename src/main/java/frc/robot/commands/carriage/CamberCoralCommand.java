package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class CamberCoralCommand extends Command{
    private final CarriageSubsystem carriageSubsystem;
    private Dimensionless speed;
    private final CamberDirection direction;

    public CamberCoralCommand(CarriageSubsystem carriageSubsystem, CamberDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = carriageSubsystem.getDefaultSpeed();
        this.direction = direction;
    }
    
    public CamberCoralCommand(CarriageSubsystem carriageSubsystem, Dimensionless speed, CamberDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = speed;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        switch (direction) {
            case LEFT:
            carriageSubsystem.moveRollersLeft(speed);
            break;
            case RIGHT:
            carriageSubsystem.moveRollersRight(speed);
            break;
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
