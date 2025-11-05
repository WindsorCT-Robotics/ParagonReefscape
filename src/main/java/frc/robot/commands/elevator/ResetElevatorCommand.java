package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;

public class ResetElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

    public ResetElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveToTargetPosition(Position.LEVEL_1); // TODO: Potentially create a duty cycle method
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.powerOff();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isRetracted(); // Left off here.
    }
}