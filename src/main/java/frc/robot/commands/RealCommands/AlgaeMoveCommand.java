package frc.robot.commands.RealCommands;

import frc.robot.hardware.MotorDirection;
import frc.robot.subsystems.AlgaeRemoverSubsystem;
import frc.robot.units.Percent;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeMoveCommand extends Command {
    private final AlgaeRemoverSubsystem motor;
    private MotorDirection direction;
    private Percent speed;

    public AlgaeMoveCommand(AlgaeRemoverSubsystem motor, MotorDirection direction, Percent speed) {
        this.motor = motor;
        this.speed = speed;
        this.direction = direction;
        addRequirements(motor);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        motor.setSpeed(speed, direction);
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