package frc.robot.commands;

import frc.robot.Units.Percent;
import frc.robot.subsystems.AlgaeRemoverSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeMoveDownCommand extends Command {
    private final AlgaeRemoverSubsystem motor;

    public AlgaeMoveDownCommand(AlgaeRemoverSubsystem motor) {
        this.motor = motor;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.moveMotor(new Percent(-0.2));
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