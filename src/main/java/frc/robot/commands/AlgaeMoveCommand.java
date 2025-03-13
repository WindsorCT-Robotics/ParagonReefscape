package frc.robot.commands;

import frc.robot.subsystems.AlgaeRemoverSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeMoveCommand extends Command {
    private final AlgaeRemoverSubsystem motor;

    public AlgaeMoveCommand(AlgaeRemoverSubsystem motor) {
        this.motor = motor;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.moveMotor(false);
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