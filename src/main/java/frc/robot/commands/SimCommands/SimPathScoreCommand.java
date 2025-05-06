package frc.robot.commands.SimCommands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Carriage.CarriageSubsystemSim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimPathScoreCommand extends SequentialCommandGroup {

    public SimPathScoreCommand(CarriageSubsystemSim rollers, CommandSwerveDrivetrain drivetrain, Limelight limelight, String direction, double level) {
        System.out.println("Pathing and Scoring...");
        addCommands(
            drivetrain.pathToAlign(limelight, false, direction),
            new SimCoralOuttakeCommand(rollers, level)
        );
    }
}