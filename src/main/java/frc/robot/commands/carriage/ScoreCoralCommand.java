package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.MotorDirection;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class ScoreCoralCommand extends Command {
    private final CarriageSubsystem carriageSubsystem;
    private final MotorDirection direction;
    private Dimensionless speed;
    
    public ScoreCoralCommand(CarriageSubsystem carriageSubsystem, MotorDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.direction = direction;
        this.speed = carriageSubsystem.getDefaultSpeed();
    }

    public ScoreCoralCommand(CarriageSubsystem carriageSubsystem, Dimensionless speed, MotorDirection direction) {
        this.carriageSubsystem = carriageSubsystem;
        this.direction = direction;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        carriageSubsystem.moveRollers(speed, direction);
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