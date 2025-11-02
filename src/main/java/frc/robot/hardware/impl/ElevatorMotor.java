package frc.robot.hardware.impl;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDistanceMotor;
import frc.robot.units.GearRatio;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorMotor implements IDistanceMotor {
    protected final SparkMax elevMotor;

    // Used https://reca.lc/linear to estimate elevator feed-forward coefficients
    private final ElevatorFeedforward ff;
    private static final Voltage STATIC_VOLTAGE = Volts.of(0.18);
    private static final Voltage GRAVITY_COMPENSATION = Volts.of(3.31);
    private static final Per<VoltageUnit, LinearVelocityUnit> VELOCITY_FF = VoltsPerMeterPerSecond.ofNative(1.53);
    private static final Per<VoltageUnit, LinearAccelerationUnit> ACCELERATTION_FF = VoltsPerMeterPerSecondSquared.ofNative(0.34);
    
    private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(123.15); // Target RPM: 5600
    private static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(123.15); // Target RPM/s^2: 5600
    private static final GearRatio GEAR_RATIO = new GearRatio(1);
    private static final Distance PULLEY_DIAMETER = Meters.of(0.042);
    private static final Distance ALLOWED_ERROR = Meters.of(0.75);
    private static final Distance DISTANCE_FROM_FLOOR = Meters.of(0.115);

    public ElevatorMotor(SparkMax motor) {
        ff = new ElevatorFeedforward(STATIC_VOLTAGE.in(Volts), GRAVITY_COMPENSATION.in(Volts), VELOCITY_FF.in(VoltsPerMeterPerSecond), ACCELERATTION_FF.in(VoltsPerMeterPerSecondSquared));
        SparkMaxConfig elevMotorConfig = new SparkMaxConfig();

        elevMotor = motor;
        
        elevMotorConfig.encoder
            .positionConversionFactor(
                PULLEY_DIAMETER
                    .times(Math.PI)
                    .times(GEAR_RATIO.asDouble())
                    .in(Meters));

        elevMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchType(Type.kNormallyOpen);

        elevMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                .p(Meters.of(.02273).in(Meters)) // 0.003 rotations
                .i(0)
                .d(0)
                .outputRange(Percent.of(-100).in(Value), Percent.of(100).in(Value));

        elevMotorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .maxVelocity(MAX_VELOCITY.in(Meters.per(Minute)))
                .maxAcceleration(MAX_ACCELERATION.in(Meters.per(Minute).per(Second)))
                .allowedClosedLoopError(ALLOWED_ERROR.in(Meters));
        
        elevMotorConfig.idleMode(IdleMode.kBrake);
        elevMotorConfig.inverted(true);
        elevMotor.configure(elevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void travelTo(Distance position) {
        elevMotor
            .getClosedLoopController()
            .setReference(position.minus(DISTANCE_FROM_FLOOR).in(Meters), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
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
        elevMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void hold() {
        elevMotor.setVoltage(ff.calculate(0));
    }

    @Override
    public void resetRelativeEncoder() {
        elevMotor.getEncoder().setPosition(0);
    }
    
    @Override
    public Distance getPosition() {
        return Meters.of(elevMotor.getEncoder().getPosition());
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(elevMotor.getAppliedOutput());
    }

    @Override
    public LinearVelocity getVelocity() {
        return Meters.per(Minute).of(elevMotor.getEncoder().getVelocity());
    }
}
