package frc.robot.hardware.impl;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDutyMotor;
import frc.robot.hardware.IRPMMotor;

public class AlgaeMotor implements IDutyMotor, IRPMMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);

    public AlgaeMotor(SparkMax motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
            
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDuty(Dimensionless speed) {
        if (speed.gt(MAX_DUTY) || speed.lt(MIN_DUTY)) {
            throw new IllegalArgumentException("Speed " + speed + " is out of bounds for duty motor Acceptable ranges are [" + MIN_DUTY + ", " + MAX_DUTY + "].");
        }
        motor.set(speed.in(Value));
    }
    
    @Override
    public Voltage getVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(encoder.getPosition());
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
        // TODO: Determine safe voltage limits for motor
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void hold() {
        motor.stopMotor();
    }

    @Override
    public void resetRelativeEncoder() {
        encoder.setPosition(0);
    }
    
    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }
}