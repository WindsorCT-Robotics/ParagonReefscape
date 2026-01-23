package frc.robot.hardware;

/**
 * Provides a common interface for a motor with limit switches.
 */
public interface ILimitedMotor extends IMotor {
    /**
     * Determine if the motor has hit its forward limit switch.
     * 
     * @return True if the forward limit switch has been hit.
     * @implNote If there is no forward limit, this function will always return
     *           False.
     */
    public boolean isAtForwardLimit();

    /**
     * Determine if the motor has hit its reverse limit switch.
     * 
     * @return True if the reverse limit switch has been hit.
     * @implNote If there is no reverse limit, this function will always return
     *           False.
     */
    public boolean isAtReverseLimit();
    
    /**
     * Moves the motor to its reverse limit switch.
     * 
     * @apiNote This function must be used in conjunction with {@link #isAtReverseLimit()} to determine when to stop the motor.
     */
    public void home();
}
