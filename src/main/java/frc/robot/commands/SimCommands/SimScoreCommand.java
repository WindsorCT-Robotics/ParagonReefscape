package frc.robot.commands.SimCommands;

import frc.robot.subsystems.carriage.CarriageSubsystemSim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimScoreCommand extends SequentialCommandGroup {

    public SimScoreCommand(CarriageSubsystemSim rollers, double level) {
        System.out.println("Socring...");
        addCommands(
            new SimCoralOuttakeCommand(rollers, level)
        );
    }
}