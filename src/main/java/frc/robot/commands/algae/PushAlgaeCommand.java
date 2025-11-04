package frc.robot.commands.algae;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.MotorDirection;
import frc.robot.subsystems.algae.AlgaeRemoverSubsystem;

public class PushAlgaeCommand extends Command {
    private final AlgaeRemoverSubsystem algaeRemoverSubsystem;
    private final Dimensionless speed;

    public PushAlgaeCommand(AlgaeRemoverSubsystem algaeRemoverSubsystem, Dimensionless speed) {
        this.algaeRemoverSubsystem = algaeRemoverSubsystem;
        this.speed = speed;
        addRequirements(algaeRemoverSubsystem);
    }

    public PushAlgaeCommand(AlgaeRemoverSubsystem algaeRemoverSubsystem) {
        this(algaeRemoverSubsystem, algaeRemoverSubsystem.getDefaultSpeed());
    }

    @Override
    public void initialize() {
        algaeRemoverSubsystem.setSpeed(speed, MotorDirection.FORWARD); //TODO: Check if this is the correct motor direction
    }

    @Override
    public void end(boolean interrupted) {
        algaeRemoverSubsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished(); // Only finishes when the user decides to.
    }
}
