package frc.robot.hardware.sim;

import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.IMotor;

/**
 * A simulated motor that is capable of setting its duty cycle to run at a desired speed.
 * @see IMotor
 * @see IDutyMotor
 */
public interface ISimSpeedMotor extends ISimHardware, IDutyMotor {

}
