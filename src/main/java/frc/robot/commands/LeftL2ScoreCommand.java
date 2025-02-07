package frc.robot.commands;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeftL2ScoreCommand extends SequentialCommandGroup {

    public LeftL2ScoreCommand(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain) {
        addCommands(
            new ParallelCommandGroup(
                new RetractElevatorCommand(elevator),
                new LeftBeamAdjustment(drivetrain)
            ),
            new OuttakeBeamCommand(rollers)
        );
    }
}