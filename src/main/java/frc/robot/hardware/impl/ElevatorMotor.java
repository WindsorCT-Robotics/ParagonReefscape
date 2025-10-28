package frc.robot.hardware.impl;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDistanceMotor;
import frc.robot.units.GearRatio;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorMotor implements IDistanceMotor {
    protected final SparkMax elevMotor;
    private final ElevatorFeedforward ff;
    private GearRatio gearRatio;
    private Angle circumference;
    private Mass elevatorWeight;
    private static final Voltage STATIC_VOLTAGE = Volts.of(0.18);

    public ElevatorMotor(SparkMax motor, GearRatio gearRatio, Angle pulleyCircumference, Mass elevatorWeight) {
        ff = new ElevatorFeedforward(0, 0, 0);
        SparkMaxConfig elevMotorConfig = new SparkMaxConfig();
        this.gearRatio = gearRatio;
        this.circumference = pulleyCircumference;
        this.elevatorWeight = elevatorWeight;

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

    public Distance getPosition() {
        return Meters.
    }

    // @Override
    // public void travelTo(Meters position) {
    //     travelTo(position.asRotations(gearRatio, circumference));
    // }

    // @Override
    // public void travelTo(Rotations position) {
    //     elevMotor
    //         .getClosedLoopController()
    //         .setReference(
    //             position.asDouble(),
    //             ControlType.kMAXMotionPositionControl,
    //             ClosedLoopSlot.kSlot0);
    // }

    @Override
    public void travelTo(Distance position) {
        elevMotor
            .getClosedLoopController()
            .setReference(position)

    }

    @Override
    public Angle getRotations() {
        return Rotations.of(elevMotor.getEncoder().getPosition());
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
    public RotationsPerMinute getVelocity() {
        return new RotationsPerMinute(elevMotor.getEncoder().getVelocity());
    }
}
