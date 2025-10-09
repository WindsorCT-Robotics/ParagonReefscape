package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IPositionalMotor;
import frc.robot.units.Centimeters;
import frc.robot.units.Meters;

public class ElevatorSubsystem extends SubsystemBase {
    private final IPositionalMotor motor;
    private static final Centimeters LEVEL2_HEIGHT = new Centimeters(81);
    // TODO: Double-check to make sure this value is correct
    private static final Centimeters LEVEL2_ALGAE_HEIGHT = new Centimeters(108);
    private static final Centimeters LEVEL3_HEIGHT = new Centimeters(121);
    private static final Centimeters LEVEL1_HEIGHT = new Centimeters(46);

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

    public ElevatorSubsystem(String subsystemName, IPositionalMotor motor) {
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
                motor.travelTo(LEVEL1_HEIGHT.asMeters());
                break;
            case LEVEL_2:
                motor.travelTo(LEVEL2_HEIGHT.asMeters());
                break;
            case LEVEL_ALGAE:
                motor.travelTo(LEVEL2_ALGAE_HEIGHT.asMeters());
                break;
            case LEVEL_3:
                motor.travelTo(LEVEL3_HEIGHT.asMeters());
                break;
            default:
                throw new InvalidElevatorPositionException(position);
        }
    }

    @AutoLogOutput(key = "Elevator/IsRetracted")
    public boolean isRetracted() {
        return (motor.isAtReverseLimit() || motor.getPosition().asCentimeters().equals(LEVEL1_HEIGHT));
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    public boolean isExtended() {
        return (motor.isAtForwardLimit() || motor.getPosition().asCentimeters().equals(LEVEL3_HEIGHT));
    }
    
    public Meters getHeight() {
        return motor.getPosition();
    }

    public void resetRelativeEncoder() {
        motor.resetRelativeEncoder();
    }
}
