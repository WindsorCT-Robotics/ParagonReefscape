package frc.robot.hardware;

import frc.robot.units.Percent;

/**
 * A motor that is capable of setting its duty cycle to run at a desired speed.
 * @see IMotor
 */
public interface ISpeedMotor extends IMotor {
    /**
     * Sets the motor to run at a specific speed.
     * @param speed The percentage of the motor power to apply to achieve the desired speed.
     * @param direction The direction to run the motor in.
     */
    public void setSpeed(Percent speed, MotorDirection direction);
}
