package frc.robot.commands.RealCommands;

import frc.robot.Units.Percent;
import frc.robot.subsystems.Carriage.CarriageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;


public class ManualMoveRollersCommand extends Command {
    private final CarriageSubsystem rollers;
    private DoubleSupplier speed;

    public ManualMoveRollersCommand(CarriageSubsystem rollers, DoubleSupplier speed) {
        this.rollers = rollers;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        rollers.manualMoveRollers(new Percent(speed.getAsDouble() / 4.5));
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