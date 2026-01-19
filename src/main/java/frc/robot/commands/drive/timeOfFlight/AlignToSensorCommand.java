package frc.robot.commands.drive.timeOfFlight;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.timeofflight.TimeOfFlightSubsystem;

public class AlignToSensorCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TimeOfFlightSubsystem timeOfFlight;
    private final LinearVelocity ALIGN_VELOCITY_X;
    private final HorizontalDirection direction;

    public AlignToSensorCommand(CommandSwerveDrivetrain drivetrain, TimeOfFlightSubsystem timeOfFlight, HorizontalDirection direction) {
        this.drivetrain = drivetrain;
        this.timeOfFlight = timeOfFlight;
        this.direction = direction;
        this.ALIGN_VELOCITY_X = MetersPerSecond.of(0.6);
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
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return timeOfFlight.isAtDistanceThreshold();
    }

    private void moveLeft() {
        drivetrain.robotCentricSwerveRequest(ALIGN_VELOCITY_X, MetersPerSecond.zero(), DegreesPerSecond.zero());
    }

    private void moveRight() {
        drivetrain.robotCentricSwerveRequest(ALIGN_VELOCITY_X.times(-1), MetersPerSecond.zero(), DegreesPerSecond.zero());
    }
}
