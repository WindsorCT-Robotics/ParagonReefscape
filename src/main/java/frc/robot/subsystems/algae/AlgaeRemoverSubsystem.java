package frc.robot.subsystems.algae;

import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.MotorDirection;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private boolean isEnabled;
    private Dimensionless targetDutyCyle;
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
        builder.addDoubleProperty("Target Duty Cycle", () -> targetDutyCyle.in(Percent), null);
        builder.addStringProperty("Target Direction", targetDirection::toString, null);
        builder.addDoubleProperty("Speed (RPM)", () -> motor.getVelocity().in(RPM), null);
    }

    public void setSpeed(Dimensionless motorPower, MotorDirection direction) {
        motor.setSpeed(motorPower, direction);

        isEnabled = true;
        targetDutyCyle = motorPower;
        targetDirection = direction;
    }

    public void stopMotor() {
        motor.stop();

        isEnabled = false;
        targetDutyCyle = Percent.of(0);
        targetDirection = MotorDirection.STOPPED;
    }
}