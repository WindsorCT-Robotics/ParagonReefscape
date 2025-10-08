package frc.robot.hardware;

import frc.robot.hardware.sim.ISimMotor;

/**
 * A simulated motor that is capable of setting its duty cycle to run at a desired speed.
 * @see IMotor
 * @see ISpeedMotor
 */
public interface ISimSpeedMotor extends ISimMotor, ISpeedMotor {

}
