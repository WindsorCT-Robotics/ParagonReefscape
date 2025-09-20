package frc.robot.hardware;

import frc.robot.Units.Voltage;
import frc.robot.Units.Rotations;

public interface IMotor {
    public void setVoltage(Voltage voltage);
    public Rotations getRotations();
    public void stop();
    public void powerOff();
    public boolean isMoving();
    
    public boolean isAtForwardLimit();
    public boolean isAtReverseLimit();
}
