package frc.robot.hardware;

import edu.wpi.first.units.measure.Voltage;

/**
 * Provides a common intorface for a basic motor.
 */
public interface IMotor {
    /**
     * Turns the motor by applying a specific Voltage
     * @param voltage The voltage to set.
     */
    public void setVoltage(Voltage voltage);

    /**
     * Gets the voltage currently being applied to the motor.
     * @return The voltage currently being applied to the motor.
     */
    public Voltage getVoltage();
    
    /**
     * Resets the recorded number of rotations of the motor.
     */
    public void resetRelativeEncoder();

    /**
     * Hold the motor at its present position, accounting for external forces like
     * end effector weight and gravity.
     */
    public void hold();

    /**
     * Stops the flow of current to the motor.
     * 
     * @apiNote When using this function, external forces may still stop the motor.
     */
    public void stop();

    /**
     * Determine if the motor is moving.
     * 
     * @return True if the motor is moving.
     */
    public boolean isMoving();
}