package frc.robot.commands;

import frc.robot.subsystems.AlgaeRemoverSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeMoveCommand extends Command {
    private final AlgaeRemoverSubsystem motor;
    private boolean reverse;
    private double speed;

    public AlgaeMoveCommand(AlgaeRemoverSubsystem motor, boolean reverse, double speed) {
        this.motor = motor;
        this.speed = speed;
        this.reverse = reverse;
        addRequirements(motor);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.moveMotor(reverse, speed);
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