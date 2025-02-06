package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class L3ScoreCommand extends SequentialCommandGroup {

    public L3ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator) {
        addCommands(
            new ExtendElevatorCommand(elevator),
            new OuttakeBeamCommand(rollers)
        );
    }
}