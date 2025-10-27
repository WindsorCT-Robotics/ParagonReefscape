package frc.robot.hardware;

import edu.wpi.first.units.measure.Dimensionless;

/**
 * A motor that is capable of setting its duty cycle to run at a desired speed.
 * @see IMotor
 */
public interface IDutyMotor extends IMotor {
    /**
     * Sets the motor to run at a specific speed.
     * @param speed The percentage of the motor power to apply to achieve the desired speed.
     * @param direction The direction to run the motor in.
     */
    public void setSpeed(Dimensionless speed, MotorDirection direction);
}
