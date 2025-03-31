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

public class AlgaeRemoverSubsystem extends SubsystemBase {
    private static final int ALGAE_MOTOR_CANID = 17;

    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private RelativeEncoder motorEncoder;

    public AlgaeRemoverSubsystem() {
        motor = new SparkMax(ALGAE_MOTOR_CANID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(false);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(50);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        
    }

    public void moveMotor(boolean reverse, double speed) {
        if (!reverse) {
            motor.set(speed);
        } else {
            motor.set(-speed);
        }
        
    }

    public void manualMoveMotor(Percent speed) {
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