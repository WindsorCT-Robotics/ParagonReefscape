package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class LoadCoral extends Command {
    private final CarriageSubsystem carriageSubsystem;
    private Dimensionless speed;

    public LoadCoral(CarriageSubsystem carriageSubsystem, Dimensionless speed) {
        this.carriageSubsystem = carriageSubsystem;
        this.speed = speed;
    }

    public LoadCoral(CarriageSubsystem carriageSubsystem) {
        this(carriageSubsystem, carriageSubsystem.getDefaultSpeed());
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
        return carriageSubsystem.isBeamBroken();
    }
}