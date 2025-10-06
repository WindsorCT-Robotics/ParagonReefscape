package frc.robot.subsystems;

import frc.robot.hardware.ISpeedMotor;
import frc.robot.hardware.MotorDirection;
import frc.robot.units.Percent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private ISpeedMotor motor;

    public AlgaeRemoverSubsystem(ISpeedMotor motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        
    }

    public void setSpeed(Percent motorPower, MotorDirection direction) {
        motor.setSpeed(motorPower, direction);
    }

    public void stopMotor() {
        motor.stop();
    }
}