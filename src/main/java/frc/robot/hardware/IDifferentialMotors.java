package frc.robot.hardware;

import frc.robot.units.Percent;

public interface IDifferentialMotors extends ISpeedMotor {
    /**
     * Move right at the specified speed.
     * @param speed
     */
    public void moveRight(Percent speed);

    /**
     * Move left at the specified speed.
     * @param speed
     */
    public void moveLeft(Percent speed);
    
    /**
     * Determine if both motors have hit their forward limits.
     */
    @Override
    public boolean isAtForwardLimit();
    
    /**
     * Determine if both motors have hit their reverse limits.
     */
    @Override
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
}
