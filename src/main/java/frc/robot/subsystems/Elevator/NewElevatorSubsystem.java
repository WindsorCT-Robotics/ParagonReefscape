package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Units.CANId;
import frc.robot.Units.Voltage;

public class NewElevatorSubsystem {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch reverseLimitSwitch;
    private final SparkLimitSwitch forwardLimitSwitch;
    private static final Voltage gravity = new Voltage(0.4);

    public NewElevatorSubsystem(SparkMax motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.forwardLimitSwitch = motor.getForwardLimitSwitch();
        this.reverseLimitSwitch = motor.getReverseLimitSwitch();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putBoolean("Is Fully Extended?", isExtended());
        SmartDashboard.putBoolean("Is Fully Retracted?", isRetracted());
    }

    public void stop() {
        motor.setVoltage(gravity.asDouble());
    }
    
    public void powerOff() {
        motor.stopMotor();
    }
    
    @AutoLogOutput(key = "Elevator/IsRetracted")
    public boolean isRetracted() {
        return reverseLimitSwitch.isPressed();
    }

    @AutoLogOutput(key = "Elevator/IsExtended")
    public boolean isExtended() {
        return forwardLimitSwitch.isPressed();
    }
}
