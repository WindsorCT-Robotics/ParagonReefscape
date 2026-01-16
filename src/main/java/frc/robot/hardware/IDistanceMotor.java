package frc.robot.hardware;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A motor that can move a certain distance relative to the robot on any one axis.
 * @see IMotor
 */
public interface IDistanceMotor extends IMotor {
    /**
     * Command the motor to travel to a specific real-world position from the robot.
     * @param position The position to move the motor relative to the real world position of the robot.
     * @see Meters
     */
    public void travelTo(Distance position);

    /**
     * Gets the current position of the motor relative to its position in the real world.
     * @return The current position of the motor in Meters relative to its position in the real world.
     * @see Meters
     */
    public Distance getPosition();
    
    /**
     * Gets the current speed of the motor.
     * 
     * @return Motor speed.
     */
    public LinearVelocity getVelocity();
}