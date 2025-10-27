package frc.robot.hardware;

import frc.robot.hardware.sim.ISimMotor;

/**
 * A positional motor under physics simulation.
 * @see IDistanceMotor
 * @see ISimMotor
 */
public interface ISimPositionalMotor extends ISimMotor, IDistanceMotor {
    
}
