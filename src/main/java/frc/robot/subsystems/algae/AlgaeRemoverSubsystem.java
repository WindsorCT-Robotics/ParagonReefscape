package frc.robot.subsystems.algae;

import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.units.Percent;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private boolean isEnabled;
    private Percent targetDutyCyle;
    private MotorDirection targetDirection;
    private IDutyMotor motor;

    public AlgaeRemoverSubsystem(String subsystemName, IDutyMotor motor) {
        super(subsystemName);

        this.motor = motor;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addBooleanProperty("IsEnabled", () -> isEnabled, null);
        builder.addDoubleProperty("Target Duty Cycle", targetDutyCyle::asDouble, null);
        builder.addStringProperty("Target Direction", targetDirection::toString, null);
        builder.addDoubleProperty("Speed (RPM)", () -> motor.getVelocity().asDouble(), null);
    }

    public void setSpeed(Percent motorPower, MotorDirection direction) {
        motor.setSpeed(motorPower, direction);

        isEnabled = true;
        targetDutyCyle = motorPower;
        targetDirection = direction;
    }

    public void stopMotor() {
        motor.stop();

        isEnabled = false;
        targetDutyCyle = new Percent(0);
        targetDirection = MotorDirection.STOPPED;
    }
}