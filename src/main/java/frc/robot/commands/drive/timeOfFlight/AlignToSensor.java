package frc.robot.commands.drive.timeOfFlight;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TimeOfFlight.TimeOfFlightSubsystem;

public class AlignToSensor extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TimeOfFlightSubsystem timeOfFlight;
    private final LinearVelocity ALIGN_VELOCITY_Y;
    private final HorizontalDirection direction;

    public AlignToSensor(CommandSwerveDrivetrain drivetrain, TimeOfFlightSubsystem timeOfFlight, HorizontalDirection direction) {
        this.drivetrain = drivetrain;
        this.timeOfFlight = timeOfFlight;
        this.direction = direction;
        this.ALIGN_VELOCITY_Y = drivetrain.getDefaultTOFSpeed();
    }

    @Override
    public void initialize() {
        switch (direction) {
            case LEFT:
                // Moving Left Command
                break;
            case RIGHT:
                // Moving Right Command
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    private boolean isAligned() {
        switch (direction) {
            case LEFT:
                return timeOfFlight.getLeftDistance().in(Millimeters) < TimeOfFlightSubsystem.getThresholdDistance().in(Millimeters);
            case RIGHT:
                return timeOfFlight.getRightDistance().in(Millimeters) < TimeOfFlightSubsystem.getThresholdDistance().in(Millimeters);
            default:
                return true;
        }
    }
}
