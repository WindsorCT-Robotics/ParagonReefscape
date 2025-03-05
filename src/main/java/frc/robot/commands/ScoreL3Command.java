package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL3Command extends SequentialCommandGroup {

    public ScoreL3Command(CarriageSubsystem rollers, ElevatorSubsystem elevator) {
        addCommands(
            new ElevatorToL3Command(elevator),
            new BeamOuttakeCommand(rollers)
        );
    }
}