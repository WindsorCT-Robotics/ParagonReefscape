package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ExtendElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ExtendElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setToTop();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.motorStop();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTop();
    }
}