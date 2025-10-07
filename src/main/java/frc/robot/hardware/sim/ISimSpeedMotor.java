package frc.robot.hardware.sim;

import frc.robot.hardware.IMotor;
import frc.robot.hardware.ISpeedMotor;

/**
 * A simulated motor that is capable of setting its duty cycle to run at a desired speed.
 * @see IMotor
 * @see ISpeedMotor
 */
public interface ISimSpeedMotor extends ISimMotor, ISpeedMotor {

}
