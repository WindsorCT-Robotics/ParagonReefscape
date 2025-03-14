package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorControlCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int level;
    private boolean reached;

    public ElevatorControlCommand(ElevatorSubsystem elevator, int level) {
        this.elevator = elevator;
        this.level = level;
        this.reached = false;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println(level);
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
        if (!reached) {
            if (this.level == 3) {
                reached = elevator.isAtL3();
            } else if (this.level == 2) {
                reached = elevator.isAtL2();
            } else {
                reached = elevator.isAtL1();
            }
            if (reached) {
                elevator.stopMotor();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}