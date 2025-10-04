package frc.robot.commands.SimCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.carriage.CarriageSubsystemSim;
import frc.robot.subsystems.Elevator.ElevatorSubsystemSim;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimPathScoreCommand extends SequentialCommandGroup {

    public SimPathScoreCommand(CarriageSubsystemSim rollers, ElevatorSubsystemSim elevator, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, double level) {
        System.out.println("Pathing and Scoring...");
        addCommands(
            new ParallelCommandGroup(
                drivetrain.pathToAlign(limelight, false, direction),
                new SimElevatorCommand(elevator, level)
            ),
            new SimCoralOuttakeCommand(rollers, level)
        );
    }
}