package frc.robot.hardware.impl.carriage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.IDutyRPMMotor;

public class CarriageMotor implements IDutyRPMMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private static final Current CURRENT_LIMIT = Amps.of(50);
    private static final Dimensionless MAX_DUTY = Percent.of(100);
    private static final Dimensionless MIN_DUTY = Percent.of(-100);
    /**
     * Defines a motor used in the carriage system. The motor will be configured with IdleMode sett to brake and a current limit of 50A.
     * @param motor Thhe SparkMax motor controller.
     * @param willRunInverted Whether the motor will be inverted. By default, the left motor is inverted.
     */
    public CarriageMotor(SparkMax motor, boolean willRunInverted) {
        this.motor = motor;
        this.encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(willRunInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void setDuty(Dimensionless speed) {
        if(speed.gt(MAX_DUTY) || speed.lt(MIN_DUTY)) {
            throw new IllegalArgumentException("Duty cycle " + speed + " is out of bounds. Valid range is [" + MIN_DUTY + ", " + MAX_DUTY + "].");
        }
        
        motor.set(speed.in(Value));
    }
}