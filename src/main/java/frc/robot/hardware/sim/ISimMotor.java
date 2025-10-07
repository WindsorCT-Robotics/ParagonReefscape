package frc.robot.hardware.sim;

import frc.robot.hardware.IMotor;

/**
 * A motor under physics simulation.
 * @see IMotor
 */
public interface ISimMotor extends IMotor {
    /**
     * Iterate a frame of the motor physics simulation.
     */
    public void iterate();
}
