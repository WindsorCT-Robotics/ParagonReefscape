package frc.robot.commands.drive.timeOfFlight;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TimeOfFlight.TimeOfFlightSubsystem;

public class AlignToSensorCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TimeOfFlightSubsystem timeOfFlight;
    private final LinearVelocity ALIGN_VELOCITY_Y;
    private final HorizontalDirection direction;
    private final RobotCentric driveRequest;

    public AlignToSensorCommand(CommandSwerveDrivetrain drivetrain, TimeOfFlightSubsystem timeOfFlight, HorizontalDirection direction) {
        this.drivetrain = drivetrain;
        this.timeOfFlight = timeOfFlight;
        this.direction = direction;
        this.ALIGN_VELOCITY_Y = drivetrain.getDefaultTOFSpeed();
        driveRequest = new RobotCentric();
    }

    @Override
    public void initialize() {
        switch (direction) {
            case LEFT:
                moveLeft();
                break;
            case RIGHT:
                moveRight();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        stop();
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

    private void moveLeft() {
        drivetrain.setControl(driveRequest.withVelocityY(ALIGN_VELOCITY_Y));
    }

    private void moveRight() {
        drivetrain.setControl(driveRequest.withVelocityY(ALIGN_VELOCITY_Y.times(-1)));
    }

    private void stop() { // TODO: Should this be a part of CommandSwerveDrivetrain?
        drivetrain.setControl(
            driveRequest
            .withVelocityX(MetersPerSecond.of(0))
            .withVelocityY(MetersPerSecond.of(0)));
    }
}
