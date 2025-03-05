package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorMoveCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int level;

    public ElevatorMoveCommand(ElevatorSubsystem elevator, int level) {
        this.elevator = elevator;
        this.level = level;
    }

    @Override
    public void initialize() {
        if (this.level == 3) {
            elevator.setToL3();
        } else if (this.level == 2) {
            elevator.setToL2();
        } else {
            elevator.setToL1();
        }
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
        if (this.level == 3) {
            return elevator.isAtL3();
        } else if (this.level == 2) {
            return elevator.isAtL2();
        } else {
            return elevator.isAtL1();
        }
    }
}