package frc.robot.hardware;

/**
 * A motor under physics simulation.
 * @see IMotor
 */
public interface ISimMotor extends IMotor {
    /**
     * Iterate a frame of the motor physics simulation.
     */
    public void iterate();
}
