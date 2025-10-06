package frc.robot.hardware;

import frc.robot.units.Meters;
import frc.robot.units.Rotations;

/**
 * A motor that can seek to a specific position.
 * @see IMotor
 */
public interface IPositionalMotor extends IMotor {
    /**
     * Command the motor to travel to a specific real-world position.
     * @param position The position to move the motor relative to the real world.
     * @see Meters
     */
    public void travelTo(Meters position);

    /**
     * Command the motor to travel to a specific number of rotations.
     * @param position The absolute position to seek to in rotations.
     * @see Rotations
     * @deprecated Meters is preferred for representing a disance.
     */
    @Deprecated(since = "Meters is preferred for representing a distance.")
    public void travelTo(Rotations position);
    
    /**
     * Gets the current position of the motor relative to its position in the real world.
     * @return The current position of the motor in Meters relative to its position in the real world.
     * @see Meters
     */
    public Meters getPosition();
}