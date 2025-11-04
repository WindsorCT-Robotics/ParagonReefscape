package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.MotorDirection;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class RepositionCoralCommand extends Command {
    private final CarriageSubsystem carriageSubsystem;
    private Dimensionless speed;
    private boolean isCoralOutOfSensorRangeOnce = false;

    public RepositionCoralCommand(CarriageSubsystem carriageSubsystem) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = carriageSubsystem.getDefaultSpeed();
    }

    public RepositionCoralCommand(CarriageSubsystem carriageSubsystem, Dimensionless speed) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        if (carriageSubsystem.isBeamBroken()) {
            carriageSubsystem.moveRollers(speed, MotorDirection.REVERSE);   
        } else {
            end(false);
        }
    }

    @Override
    public void execute() {
        if (!carriageSubsystem.isBeamBroken()) {
            isCoralOutOfSensorRangeOnce = true;
            carriageSubsystem.moveRollers(speed, MotorDirection.FORWARD);
        }
    }

    @Override
    public void end(boolean interrupted) {
        carriageSubsystem.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return carriageSubsystem.isBeamBroken() && isCoralOutOfSensorRangeOnce;
    }
}
