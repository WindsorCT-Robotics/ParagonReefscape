package frc.robot.hardware.impl;

import static edu.wpi.first.units.Units.Millimeters;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.IDistanceSensor;

public class FusionTimeOfFlightSensor implements IDistanceSensor {
    private TimeOfFlight sensor;

    public FusionTimeOfFlightSensor(TimeOfFlight sensor) {
        this.sensor = sensor;
    }

    @Override
    public Distance getDistance() {
        return Millimeters.of(sensor.getRange());
    }
}
