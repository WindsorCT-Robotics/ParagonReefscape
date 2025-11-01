package frc.robot.hardware.impl;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.IRPMMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.hardware.exceptions.InvalidMotorDirectionException;

public class AlgaeMotor implements IDutyMotor, IRPMMotor {
    private final SparkMax motor;

    public AlgaeMotor(SparkMax motor) {
        this.motor = motor;

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
            
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        default:
            throw new InvalidMotorDirectionException();
        }
    }
    
    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(motor.getEncoder().getPosition());
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
    public boolean isMoving() {
        return (getVelocity().gt(RPM.zero()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void hold() {
        motor.stopMotor();
    }

    @Override
    public void resetRelativeEncoder() {
        motor.getEncoder().setPosition(0);
    }
    
    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }
    
    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.set(velocity.in(RPM));
    }
}