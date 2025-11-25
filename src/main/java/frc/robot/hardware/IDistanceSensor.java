package frc.robot.hardware;

import edu.wpi.first.units.measure.Distance;

public interface IDistanceSensor extends ISensor {

    /**
     * Gets the distance reading from the sensor.
     * @return The distance.
     */
    public Distance getDistance();
}
