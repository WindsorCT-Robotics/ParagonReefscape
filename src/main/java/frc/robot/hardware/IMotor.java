package frc.robot.hardware;

import frc.robot.units.Voltage;
import frc.robot.units.Rotations;
import frc.robot.units.RotationsPerMinute;

/**
 * A motor for driving components in a robot.
 */
public interface IMotor {
    /**
     * Turns the motor by applying a specific Voltage
     * @param voltage The voltage to set.
     */
    public void setVoltage(Voltage voltage);

    /**
     * Gets the position of the motor in rotations.
     * @return The position of the motor in rotations.
     */
    public Rotations getRotations();

    /**
     * Resets the recorded number of rotations of the motor.
     */
    public void resetRelativeEncoder();

    /**
     * Hold the motor at its present position, accounting for external forces like end effector weight and gravity.
     */
    public void hold();
    
    /**
     * Stops the flow of current to the motor.
     * @apiNote When using this function, external forces may still stop the motor.
     */
    public void stop();
    
    /**
     * Determine if the motor is moving.
     * @return True if the motor is moving.
     */
    public boolean isMoving();

    /**
     * Gets the current speed of the motor in RPM.
     * @return Motor speed in RPM.
     */
    public RotationsPerMinute getRPM();
    
    /**
     * Determine if the motor has hit its forward limit switch.
     * @return True if the forward limit switch has been hit.
     * @implNote If there is no forward limit, this function will always return False.
     */
    public boolean isAtForwardLimit();

    /**
     * Determine if the motor has hit its reverse limit switch.
     * @return True if the reverse limit switch has been hit.
     * @implNote If there is no reverse limit, this function will always return False.
     */
    public boolean isAtReverseLimit();
}