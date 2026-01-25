package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hardware.ILimitedDistanceMotor;

public class ElevatorSubsystem extends SubsystemBase {
    private final ILimitedDistanceMotor motor;
    private static final Distance LEVEL2_HEIGHT = Centimeters.of(81);
    // TODO: Double-check to make sure this value is correct
    private static final Distance LEVEL2_ALGAE_HEIGHT = Centimeters.of(108);
    private static final Distance LEVEL3_HEIGHT = Centimeters.of(121);
    private static final Distance LEVEL1_HEIGHT = Centimeters.of(46);
    private static final Time DEBOUNCE_TIME = Milliseconds.of(50);
    public final Trigger isFullyExtended;
    public final Trigger isFullyRetracted;

    public enum Position {
        LEVEL_1,
        LEVEL_2,
        LEVEL_ALGAE,
        LEVEL_3
    }

    public ElevatorSubsystem(String subsystemName, ILimitedDistanceMotor motor) {
        super(subsystemName);
        this.motor = motor;
        this.isFullyExtended = new Trigger(motor::isAtForwardLimit).debounce(DEBOUNCE_TIME.in(Seconds), DebounceType.kBoth);
        this.isFullyRetracted = new Trigger(motor::isAtReverseLimit).debounce(DEBOUNCE_TIME.in(Seconds), DebounceType.kBoth);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Is Fully Extended?", this::isExtended, null);
        builder.addBooleanProperty("Is Fully Retracted?", this::isRetracted, null);
        builder.addDoubleProperty("Height (m)", () -> getHeight().in(Meters), this::setHeight);
    }

    private Command move(Supplier<Distance> height) {
        return runEnd(() -> motor.travelTo(height.get()), motor::hold);
    }
    
    private Distance getDistance(Position position) {
        switch (position) {
            case LEVEL_1:
                return LEVEL1_HEIGHT;
            case LEVEL_2:
                return LEVEL2_HEIGHT;
            case LEVEL_ALGAE:
                return LEVEL2_ALGAE_HEIGHT;
            case LEVEL_3:
                return LEVEL3_HEIGHT;
            default:
                throw new IllegalArgumentException("Invalid elevator position: " + position);
        }
    }
    
    public Command move(Position position) {
        return move(() -> getDistance(position));
    }

    @AutoLogOutput(key = "Elevator/IsRetracted")
    private boolean isRetracted() {
        return (motor.isAtReverseLimit() || motor.getPosition().equals(LEVEL1_HEIGHT));
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    private boolean isExtended() {
        return (motor.isAtForwardLimit() || motor.getPosition().equals(LEVEL3_HEIGHT));
    }

    @AutoLogOutput(key = "Elevator/Height")
    private Distance getHeight() {
        return motor.getPosition();
    }

    private void setHeight(double heightMeters) {
        runOnce(() -> motor.travelTo(Meters.of(heightMeters))).schedule();
    }

    public Command reset() {
        return runOnce(motor::resetRelativeEncoder);
    }
    
    public Command hold() {
        return runOnce(motor::hold);
    }

    public Command stop() {
        return runOnce(motor::stop);
    }

    public Command translateElevator(Position position) {
        return run(() -> move(position));
    }
    
    public Command home() {
        return run(() -> motor.home())
            .until(motor::isAtReverseLimit)
            .andThen(motor::resetRelativeEncoder);
    }
}
