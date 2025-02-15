package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Units.Percent;
import frc.robot.subsystems.CarriageSubsystem;

public class ManualIntakeOuttakeCommand extends Command {
    private final CarriageSubsystem rollers;
    private DoubleSupplier speed;

    public ManualIntakeOuttakeCommand(CarriageSubsystem rollers, DoubleSupplier speed) {
        this.rollers = rollers;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        rollers.manualMoveIntakeRollers(new Percent(speed.getAsDouble() / 4.5));
        rollers.manualMoveOuttakeRollers(new Percent(speed.getAsDouble() / 4.5));
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stopRollers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}