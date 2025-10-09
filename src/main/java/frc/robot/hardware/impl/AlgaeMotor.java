package frc.robot.hardware.impl;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.hardware.ISpeedMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.hardware.exceptions.InvalidMotorDirectionException;
import frc.robot.units.Percent;
import frc.robot.units.Rotations;
import frc.robot.units.RotationsPerMinute;
import frc.robot.units.Voltage;

public class AlgaeMotor implements ISpeedMotor {
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
    public void setSpeed(Percent speed, MotorDirection direction) {
        switch (direction) {
        case FORWARD:
            motor.set(speed.asDouble());
            break;
        case REVERSE:
            motor.set(-speed.asDouble());
            break;
        default:
            throw new InvalidMotorDirectionException();
        }
    }

    @Override
    public Rotations getRotations() {
        return new Rotations(motor.getEncoder().getPosition());
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
        return (motor.getEncoder().getVelocity() > 0);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage.asDouble());
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
    public RotationsPerMinute getRPM() {
        return new RotationsPerMinute(motor.getEncoder().getVelocity());
    }
}