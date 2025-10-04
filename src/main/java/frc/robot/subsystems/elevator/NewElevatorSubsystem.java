package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.units.CANId;
import frc.robot.units.Centimeters;
import frc.robot.units.Meters;
import frc.robot.hardware.IPositionalMotor;

public class InvalidElevatorPositionException extends RuntimeException {
    private final int position;

    public InvalidElevatorPositionException(int position) {
        this.position = position;

        super(String.format("The given position %d is not a valid elevator position.", position));
    }

    public int getInvalidPosition() { return position; }
}

public enum ElevatorPosition {
    LEVEL_1,
    LEVEL_2,
    LEVEL_ALGAE,
    LEVEL_3
}

public class NewElevatorSubsystem extends SubsystemBase {
    private final IPositionalMotor motor;
    private static final Centimeters LEVEL2_HEIGHT = new Centimeters(81);
    // TODO: Double-check to make sure this value is correct
    private static final Centimeters LEVEL2_ALGAE_HEIGHT = new Centimeters(108);
    private static final Centimeters LEVEL3_HEIGHT = new Centimeters(121);
    private static final Centimeters LEVEL1_HEIGHT = new Centimeters(46);

    public NewElevatorSubsystem(IPositionalMotor motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Fully Extended?", isExtended());
        SmartDashboard.putBoolean("Is Fully Retracted?", isRetracted());
    }

    public void stop() {
        motor.stop();
    }

    public void powerOff() {
        motor.powerOff();
    }

    public void moveToTargetPosition(ElevatorPosition position) {
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
                break;
        }
    }

    @AutoLogOutput(key = "Elevator/IsRetracted")
    public boolean isRetracted() {
        return (motor.isAtReverseLimit() || motor.getPosition().asCentimeters().equals(LEVEL1_HEIGHT));
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    public boolean isExtended() {
        return (forwardLimitSwitch.isPressed() || motor.getPosition().asCentimeters().equals(LEVEL3_HEIGHT));
    }
}
