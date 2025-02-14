package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class RetractElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    public RetractElevatorCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevator.setToBottom();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.motorStop();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtBottom();
    }
}