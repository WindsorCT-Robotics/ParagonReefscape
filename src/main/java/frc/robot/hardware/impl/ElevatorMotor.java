package frc.robot.hardware.impl;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.hardware.IPositionalMotor;
import frc.robot.units.GearRatio;
import frc.robot.units.Kilograms;
import frc.robot.units.Meters;
import frc.robot.units.Radians;
import frc.robot.units.Rotations;
import frc.robot.units.RotationsPerMinute;
import frc.robot.units.Voltage;

public class ElevatorMotor implements IPositionalMotor {
    protected final SparkMax elevMotor;
    private final GearRatio gearRatio;
    private final Radians circumference;
    private final Kilograms elevatorWeight;
    private static final double FORCE_OF_GRAVITY = 9.81;
    private static final double TORQUE_CONSTANT = 0.018;
    private static final double PHASE_RESISTANCE = 0.03;
    private Voltage gravityCompensation;

    public ElevatorMotor(SparkMax motor, GearRatio gearRatio, Radians pulleyCircumference, Kilograms elevatorWeight) {
        SparkMaxConfig elevMotorConfig = new SparkMaxConfig();
        this.gearRatio = gearRatio;
        this.circumference = pulleyCircumference;
        this.elevatorWeight = elevatorWeight;
        this.gravityCompensation = getHoldVoltage();

        elevMotor = motor;

        elevMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchType(Type.kNormallyOpen);

        elevMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                .p(0.3) // 0.003
                .i(0)
                .d(0)
                .outputRange(-1, 1);

        elevMotorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .maxVelocity(5600)
                .maxAcceleration(5600)
                .allowedClosedLoopError(0.1);
        
        elevMotorConfig.idleMode(IdleMode.kBrake);
        elevMotorConfig.inverted(true);
        elevMotor.configure(elevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Voltage getHoldVoltage() {
        return new Voltage(
            (
                (elevatorWeight.asDouble() * FORCE_OF_GRAVITY * circumference.asDouble())
                / (gearRatio.asDouble() * TORQUE_CONSTANT)
            ) * PHASE_RESISTANCE);
    }

    public Meters getPosition() {
        return getRotations().asMeters(gearRatio, circumference);
    }

    @Override
    public void travelTo(Meters position) {
        travelTo(position.asRotations(gearRatio, circumference));
    }

    @Override
    public void travelTo(Rotations position) {
        elevMotor
            .getClosedLoopController()
            .setReference(
                position.asDouble(),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                gravityCompensation.asDouble());
    }

    @Override
    public Rotations getRotations() {
        return new Rotations(elevMotor.getEncoder().getPosition());
    }

    @Override
    public boolean isAtForwardLimit() {
        return elevMotor.getForwardLimitSwitch().isPressed();
    }

    @Override
    public boolean isAtReverseLimit() {
        return elevMotor.getReverseLimitSwitch().isPressed();
    }

    @Override
    public boolean isMoving() {
        return elevMotor.getEncoder().getVelocity() > 0;
    }

    @Override
    public void stop() {
        elevMotor.stopMotor();
    }

    @Override
    public void setVoltage(Voltage voltage) {
        elevMotor.setVoltage(voltage.asDouble());
    }

    @Override
    public void hold() {
        elevMotor.setVoltage(gravityCompensation.asDouble());
    }

    @Override
    public void resetRelativeEncoder() {
        elevMotor.getEncoder().setPosition(0);
    }
    
    @Override
    public RotationsPerMinute getRPM() {
        return new RotationsPerMinute(elevMotor.getEncoder().getVelocity());
    }
}
