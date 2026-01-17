package frc.robot.subsystems.timeofflight;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.IDistanceSensor;

public class TimeOfFlightSubsystem extends SubsystemBase {
    private final IDistanceSensor sensor;
    private static final Distance THRESHOLD_DISTANCE = Millimeters.of(400);

    public TimeOfFlightSubsystem(IDistanceSensor sensor) {
        this.sensor = sensor;
    }

    public boolean isAtDistanceThreshold() {
        return sensor.getDistance().in(Millimeters) >= THRESHOLD_DISTANCE.in(Millimeters);
    }
}
