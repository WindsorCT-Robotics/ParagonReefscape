package frc.robot.commands;

import frc.robot.subsystems.AlgaeRemoverSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeAutoMoveCommand extends Command {
    private final AlgaeRemoverSubsystem motor;

    public AlgaeAutoMoveCommand(AlgaeRemoverSubsystem motor) {
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

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}