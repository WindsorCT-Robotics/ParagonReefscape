package frc.robot.commands.RealCommands;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorResetCommand extends Command {
    private final ElevatorSubsystem elevator;

    public ElevatorResetCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.moveMotor();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotor();
        elevator.resetRelativeEncoder();
    }

    @Override
    public boolean isFinished() {
        return elevator.getLowerLimit();
    }
}