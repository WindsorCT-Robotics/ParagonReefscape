package frc.robot.subsystems.algae;

import frc.robot.hardware.ISpeedMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.units.Percent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private ISpeedMotor motor;

    public AlgaeRemoverSubsystem(ISpeedMotor motor) {
        this.motor = motor;
    }

    public void setSpeed(Percent motorPower, MotorDirection direction) {
        motor.setSpeed(motorPower, direction);
    }

    public void stopMotor() {
        motor.stop();
    }
}