package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;

public class TranslateElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private Position position;
    
    public TranslateElevatorCommand(ElevatorSubsystem elevatorSubsystem, Position position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveToTargetPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isPositionAt(position);
    }
}
