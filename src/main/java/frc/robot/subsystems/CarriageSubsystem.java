package frc.robot.subsystems;

import frc.robot.Units.Percent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CarriageSubsystem extends SubsystemBase {
    // private static final int ROLLER_MOTOR_CANID = 15;
    private static final int INTAKE_MOTOR_CANID = 16;
    private static final int OUTTAKE_MOTOR_CANID = 15;
    private static final int BEAM_BREAKER_PIN = 0;

    private SparkMax rollers;
    private SparkMax outtakeRollers;
    private SparkMax intakeRollers;
    private SparkMaxConfig rollersConfig;
    private final DigitalInput beamBreaker;
    private RelativeEncoder rollerEncoder;
    private RelativeEncoder outtakeEncoder;
    private RelativeEncoder intakeEncoder;

    private Percent speed;

    public CarriageSubsystem() {
        speed = new Percent(0.2);
        outtakeRollers = new SparkMax(OUTTAKE_MOTOR_CANID, MotorType.kBrushless);
        intakeRollers = new SparkMax(INTAKE_MOTOR_CANID, MotorType.kBrushless);
        rollersConfig = new SparkMaxConfig();

        rollersConfig.inverted(false);
        rollersConfig.idleMode(IdleMode.kBrake);

        // rollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outtakeRollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);
        // rollerEncoder = rollers.getEncoder();
        outtakeEncoder = outtakeRollers.getEncoder();
        intakeEncoder = intakeRollers.getEncoder();

        SmartDashboard.putNumber("Roller Speed RPM", 10);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker", beamBreaker.get());
        
    }

    // public void moveRollers(Percent speed) {
    //     rollers.set(speed.asDouble());  
    // }

    // public void manualMoveRollers(Percent speed) {
    //     intakeRollers.set(speed.asDouble());
    // }
    
    public void moveOuttakeRollers() {
        outtakeRollers.set(speed.asDouble());
    }

    public void manualMoveOuttakeRollers(Percent speed) {
        outtakeRollers.set(speed.asDouble());
    }

    public void moveIntakeRollers() {
        intakeRollers.set(speed.asDouble());
    }

    public void manualMoveIntakeRollers(Percent speed) {
        intakeRollers.set(speed.asDouble());
    }

    public boolean isBeamBroken() {
        return !beamBreaker.get();
    }

    // public double getRollerPosition() {
    //     return rollerEncoder.getPosition();
    // }

    public double getIntakeRollerPosition() {
        return intakeEncoder.getPosition(); 
    }

    public double getOuttakeRollerPosition() {
        return outtakeEncoder.getPosition();
    }

    public void resetRollerEncoder() {
        // rollerEncoder.setPosition(0.0);
        outtakeEncoder.setPosition(0);
        intakeEncoder.setPosition(0);   
    }
    
    public void stopRollers() {
        // rollers.stopMotor();

        intakeRollers.stopMotor();
        outtakeRollers.stopMotor();
    }

    public void stopIntakeRollers() {
        intakeRollers.stopMotor();
    }

    public void stopOuttakeRollers() {
        outtakeRollers.stopMotor();
    }
}