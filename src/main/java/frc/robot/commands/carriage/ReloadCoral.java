package frc.robot.commands.carriage;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.carriage.CarriageSubsystem;

public class ReloadCoral extends SequentialCommandGroup {
    public ReloadCoral(CarriageSubsystem carriageSubsystem, Dimensionless speed) {
        addCommands(
                new UnloadCoral(carriageSubsystem, speed),
                new LoadCoral(carriageSubsystem, speed));
    }

    public ReloadCoral(CarriageSubsystem carriageSubsystem) {
        this(carriageSubsystem, carriageSubsystem.getDefaultSpeed());
    }
}
