package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;

public class PositionElevatorCommand extends SequentialCommandGroup {
    public PositionElevatorCommand(ElevatorSubsystem elevatorSubsystem, Position position) {
        addCommands(
            new TranslateElevatorCommand(elevatorSubsystem, position),
            new HoldElevatorCommand(elevatorSubsystem)
        );
    }
}
