package frc.robot.hardware;

import frc.robot.Units.Meters;
import frc.robot.Units.Rotations;

public interface IPositionalMotor extends IMotor {
    public void travelTo(Meters position);

    @Deprecated(since = "Meters is preferred for representing a distance.")
    public void travelTo(Rotations position);
    
    public Meters getPosition();
}
