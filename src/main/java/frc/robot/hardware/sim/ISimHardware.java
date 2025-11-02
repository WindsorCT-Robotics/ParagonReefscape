package frc.robot.hardware.sim;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IMotor;

/**
 * A motor under physics simulation.
 * @see IMotor
 */
public interface ISimHardware {
    /**
     * Iterate a frame of the physics simulation.
     */
    public void iterate(Voltage batteryVoltage, Time stepTime);
}
