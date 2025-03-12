package frc.robot.subsystems;

import frc.robot.Units.Percent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private static final int ALGAE_MOTOR_CANID = 17;

    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private RelativeEncoder motorEncoder;

    private Percent speed;


    public AlgaeRemoverSubsystem() {
        speed = new Percent(0.2);
        motor = new SparkMax(ALGAE_MOTOR_CANID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(false);
        motorConfig.idleMode(IdleMode.kBrake);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorEncoder = motor.getEncoder();

        SmartDashboard.putNumber("Algae Motor Speed RPM", 10);
    }

    @Override
    public void periodic() {
        
    }

    public void moveMotor(Percent speed) {
        motor.set(speed.asDouble());
    }

    public double getMotorPosition() {
        return motorEncoder.getPosition();
    }

    public void resetRollerEncoder() {
        motorEncoder.setPosition(0);
    }
    
    public void stopMotor() {
        motor.stopMotor();
    }
}