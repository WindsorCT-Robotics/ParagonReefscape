package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RightL3ScoreCommand extends SequentialCommandGroup {

    public RightL3ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new ParallelCommandGroup(
                new ExtendElevatorCommand(elevator),
                new RightBeamAdjustment(drivetrain)
            ),
            new OuttakeBeamCommand(rollers),
            new RetractElevatorCommand(elevator)
        );
    }
}