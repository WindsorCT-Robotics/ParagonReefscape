package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Centimeters;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IDistanceMotor;

public class ElevatorSubsystem extends SubsystemBase {
    private final IDistanceMotor motor;
    private static final Distance LEVEL2_HEIGHT = Centimeters.of(81);
    // TODO: Double-check to make sure this value is correct
    private static final Distance LEVEL2_ALGAE_HEIGHT = Centimeters.of(108);
    private static final Distance LEVEL3_HEIGHT = Centimeters.of(121);
    private static final Distance LEVEL1_HEIGHT = Centimeters.of(46);

    public class InvalidElevatorPositionException extends RuntimeException {
        private final Position position;

        public InvalidElevatorPositionException(Position position) {
            super(String.format("The given position %d is not a valid elevator position.", position.ordinal()));

            this.position = position;
        }

        public Position getInvalidPosition() {
            return position;
        }
    }

    public enum Position {
        LEVEL_1,
        LEVEL_2,
        LEVEL_ALGAE,
        LEVEL_3
    }

    public ElevatorSubsystem(String subsystemName, IDistanceMotor motor) {
        super(subsystemName);
        this.motor = motor;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Is Fully Extended?", this::isExtended, null);
        builder.addBooleanProperty("Is Fully Retracted?", this::isRetracted, null);
    }

    public void stop() {
        motor.hold();
    }

    public void powerOff() {
        motor.stop();
    }

    public void moveToTargetPosition(Position position) {
        switch (position) {
            case LEVEL_1:
                motor.travelTo(LEVEL1_HEIGHT);
                break;
            case LEVEL_2:
                motor.travelTo(LEVEL2_HEIGHT);
                break;
            case LEVEL_ALGAE:
                motor.travelTo(LEVEL2_ALGAE_HEIGHT);
                break;
            case LEVEL_3:
                motor.travelTo(LEVEL3_HEIGHT);
                break;
            default:
                throw new InvalidElevatorPositionException(position);
        }
    }

    @AutoLogOutput(key = "Elevator/IsRetracted")
    public boolean isRetracted() {
        return (motor.isAtReverseLimit() || motor.getPosition().equals(LEVEL1_HEIGHT));
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    public boolean isExtended() {
        return (motor.isAtForwardLimit() || motor.getPosition().equals(LEVEL3_HEIGHT));
    }
    
    public Distance getHeight() {
        return motor.getPosition();
    }

    public void resetRelativeEncoder() {
        motor.resetRelativeEncoder();
    }
}
