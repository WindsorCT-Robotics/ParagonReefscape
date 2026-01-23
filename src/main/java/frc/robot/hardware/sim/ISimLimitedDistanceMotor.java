package frc.robot.hardware.sim;

import frc.robot.hardware.IDistanceMotor;
import frc.robot.hardware.ILimitedDistanceMotor;

/**
 * A positional motor with limit switches and homing capability under physics simulation.
 * @see IDistanceMotor
 * @see ILimitedMotor
 * @see ISimHardware
 */
public interface ISimLimitedDistanceMotor extends ISimHardware, ILimitedDistanceMotor {
    
}
