package frc.robot.subsystems.TimeOfFlight;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IDistanceSensor;

public class TimeOfFlightSubsystem extends SubsystemBase {
    private IDistanceSensor rightSensor;
    private IDistanceSensor leftSensor;
    private static final Distance THRESHOLD_DISTANCE = Millimeters.of(400);

    public TimeOfFlightSubsystem(IDistanceSensor rightSensor, IDistanceSensor leftSensor) {
        this.rightSensor = rightSensor;
        this.leftSensor = leftSensor;
    }

    public Distance getRightDistance() {
        return rightSensor.getDistance();
    }

    public Distance getLeftDistance() {
        return leftSensor.getDistance();
    }

    public static Distance getThresholdDistance() {
        return THRESHOLD_DISTANCE;
    }
}
