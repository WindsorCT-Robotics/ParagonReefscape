package frc.robot.hardware;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IRPMMotor extends IMotor {
    public Angle getPosition();
    public AngularVelocity getVelocity();
}
