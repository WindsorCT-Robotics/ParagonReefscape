package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRemoverSubsystem;

public class MoveAlgaeCommand extends Command {
    private final AlgaeRemoverSubsystem motor;

    public MoveAlgaeCommand(AlgaeRemoverSubsystem motor) {
        this.motor = motor;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.moveMotor();
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