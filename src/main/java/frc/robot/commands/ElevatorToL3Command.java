package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToL3Command extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorToL3Command(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setToL3();
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
        return elevator.isAtL3();
    }
}