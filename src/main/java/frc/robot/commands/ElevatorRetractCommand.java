package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorRetractCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorRetractCommand(ElevatorSubsystem elevator) {
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