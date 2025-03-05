package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToL1Command extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorToL1Command(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setToL1();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtL1();
    }
}