package frc.robot.hardware;

import edu.wpi.first.units.measure.Dimensionless;

/**
 * Provides a common interface for a pair of differential motors.
 */
public interface IDifferentialMotors {
    /**
     * Move right at the specified speed.
     * @param duty The speed at which the motors will move right. The left motor will move forward and the right motor will move reverse.
     */
    public void moveRight(Dimensionless duty);

    /**
     * Move left at the specified speed.
     * @param duty The speed at which the motors will move left. The right motor will move forward and the left motor will move reverse.
     */
    public void moveLeft(Dimensionless duty);
    
    /**
     * Move both motors at the specified speed.
     * @param duty The speed at which the motors will move.
     * @param direction The direction specified to move the motors.
     */
    public void move(Dimensionless duty, MotorDirection direction);
    
    /**
     * Get the left motor.
     * @return A direct reference to the left motor.
     */
    public IRPMMotor getLeftMotor();
    
    /**
     * Get the right motor.
     * @return A direct reference to the right motor.
     */
    public IRPMMotor getRightMotor();
    
    /**
     * Determine if both motors have hit their forward limits.
     */
    public boolean isAtForwardLimit();
    
    /**
     * Determine if both motors have hit their reverse limits.
     */
    public boolean isAtReverseLimit();

    /** 
     * Determine if motors have hit the far left limit.
     * @return True if motors have hit their far left limit.
     */
    public boolean isAtLeftLimit();

    /**
     * Determine if motors have hit the far right limit.
     * @return True if motors have hit their far right limit.
     */
    public boolean isAtRightLimit();
    
    /**
     * Resets the relative position of both motor encoders.
     */
    public void resetRelativeEncoder();

    /**
     * Stop both motors.
     */
    public void stop();
}
