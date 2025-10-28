package frc.robot.subsystems.algae;

import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.MotorDirection;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

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
        builder.addDoubleProperty("Target Duty Cycle (%)", () -> targetDutyCyle.in(Percent), this::setDuty);
        builder.addStringProperty("Target Direction", targetDirection::toString, null);
        builder.addDoubleProperty("Speed (RPM)", () -> motor.getVelocity().in(RPM), null);
        builder.addDoubleProperty("Voltage (v)", () -> motor.getVoltage().in(Volts), v -> motor.setVoltage(Volts.of(v)));
    }

    private void setDuty(double duty) {
        if (duty > 1 || duty < -1) {
            throw new IllegalArgumentException("Duty cycle must be between -1 and 1; got " + duty);
        }
        if (duty == 0) {
            stopMotor();
        }
        else if (duty < 0) {
            setSpeed(Percent.of(duty), MotorDirection.REVERSE);
        }
        else {
            setSpeed(Percent.of(duty), MotorDirection.FORWARD);
        }
    }
    
    public void setSpeed(Dimensionless motorPower, MotorDirection direction) {
        motor.setDuty(motorPower, direction);

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