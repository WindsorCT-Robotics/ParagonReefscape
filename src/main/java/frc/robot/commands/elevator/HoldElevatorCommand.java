package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class HoldElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

    public HoldElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.stop();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.powerOff();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
