package frc.robot.commands;

import frc.robot.Units.Percent;
import frc.robot.subsystems.AlgaeRemoverSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;


public class AlgaeManualMoveCommand extends Command {
    private final AlgaeRemoverSubsystem motor;
    private DoubleSupplier speed;

    public AlgaeManualMoveCommand(AlgaeRemoverSubsystem motor, DoubleSupplier speed) {
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.manualMoveMotor(new Percent(speed.getAsDouble() / 10));
    }

    @Override
    public void end(boolean interrupted) {
        motor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}