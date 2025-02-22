package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Units.Percent;
import frc.robot.subsystems.AlgaeRemoverSubsystem;
import frc.robot.subsystems.CarriageSubsystem;

public class ManualMoveAlgaeCommand extends Command {
    private final AlgaeRemoverSubsystem motor;
    private DoubleSupplier speed;

    public ManualMoveAlgaeCommand(AlgaeRemoverSubsystem motor, DoubleSupplier speed) {
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