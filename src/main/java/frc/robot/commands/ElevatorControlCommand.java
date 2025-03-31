package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorControlCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int level;

    public ElevatorControlCommand(ElevatorSubsystem elevator, int level) {
        this.elevator = elevator;
        this.level = level;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println(level);
        if (this.level == 3) {
            elevator.setToL3();
        } else if (this.level == 2.5) {
            elevator.setToL2_5();
        } else if (this.level == 2) {
            elevator.setToL2();
        } else {
            elevator.setToL1();
        }
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) {
        if (level == 1) {
            elevator.stopMotor();
        } else {
            elevator.holdPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}