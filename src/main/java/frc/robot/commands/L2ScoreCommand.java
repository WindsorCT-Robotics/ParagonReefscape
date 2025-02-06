package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class L2ScoreCommand extends SequentialCommandGroup {

    public L2ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator) {
        addCommands(
            new RetractElevatorCommand(elevator),
            new OuttakeBeamCommand(rollers)
        );
    }
}