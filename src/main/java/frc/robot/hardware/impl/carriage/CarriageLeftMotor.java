package frc.robot.hardware.impl.carriage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDutyRPMMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.hardware.exceptions.InvalidMotorDirectionException;

public class CarriageLeftMotor implements IDutyRPMMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private static final Current CURRENT_LIMIT = Amps.of(50);

    public CarriageLeftMotor(SparkMax motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
    }
    
    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public void resetRelativeEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void hold() {
        stop();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public boolean isMoving() {
        return encoder.getVelocity() > 0;
    }

    @Override
    public boolean isAtForwardLimit() {
        return false;
    }

    @Override
    public boolean isAtReverseLimit() {
        return false;
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(encoder.getPosition());
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    @Override
    public void setDuty(Dimensionless speed, MotorDirection direction) {
        switch (direction) {
        case FORWARD:
            motor.set(speed.in(Value));
            break;
        case REVERSE:
            motor.set(-speed.in(Value));
            break;
        case STOPPED:
            stop();
            break;
        default:
            throw new InvalidMotorDirectionException(direction);
        }
    }
}