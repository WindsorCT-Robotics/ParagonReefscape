package frc.robot.commands;

import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoScoreL1Command extends SequentialCommandGroup {

    public AutoScoreL1Command(CarriageSubsystem rollers, ElevatorSubsystem elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController op, CommandXboxController drive, String direction) {
        if (direction.equalsIgnoreCase("left")) {
            addCommands(
            new ParallelCommandGroup(
                drivetrain.pathToAlign(limelight, false, direction),
                new ElevatorToL1Command(elevator),
                new BeamLeftAdjustment(drivetrain)
            ),
            new BeamOuttakeCommand(rollers)
            );   
        }

        if (direction.equalsIgnoreCase("right")) {
            addCommands(
            new ParallelCommandGroup(
                drivetrain.pathToAlign(limelight, false, direction),
                new ElevatorToL1Command(elevator),
                new BeamRightAdjustment(drivetrain)
            ),
            new BeamOuttakeCommand(rollers)
            );   
        }
    }
}