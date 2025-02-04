package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;
import frc.robot.Units.RotationsPerMinute;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CarriageSubsystem extends SubsystemBase {
    // private static final int ROLLER_MOTOR_CANID = 15;
    private static final int OUTTAKE_MOTOR_CANID = 15;
    private static final int INTAKE_MOTOR_CANID = 16;
    private static final int BEAM_BREAKER_PIN = 2;

    private SparkMax rollers;
    private SparkMax outtakeRollers;
    private SparkMax intakeRollers;
    private SparkMaxConfig rollersConfig;
    private final DigitalInput beamBreaker;
    private RelativeEncoder rollerEncoder;
    private RelativeEncoder outtakeEncoder;
    private RelativeEncoder intakeEncoder;
    
    private Percent speed;

    private static final int MOTOR_MAX_RPM = 0;
    private static final double MOTOR_GEAR_RATIO = 0;

    public static RotationsPerMinute targetRPM = new RotationsPerMinute(0, 0);

    public CarriageSubsystem() {
        speed = new Percent(0.0);
        outtakeRollers = new SparkMax(OUTTAKE_MOTOR_CANID, MotorType.kBrushless);
        intakeRollers = new SparkMax(INTAKE_MOTOR_CANID, MotorType.kBrushless);
        rollersConfig = new SparkMaxConfig();

        rollersConfig.inverted(true);
        rollersConfig.idleMode(IdleMode.kBrake);

        // rollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outtakeRollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeRollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);
        // rollerEncoder = rollers.getEncoder();
        outtakeEncoder = outtakeRollers.getEncoder();
        intakeEncoder = intakeRollers.getEncoder();

        SmartDashboard.putNumber("Outtake Percent Speed", CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
        SmartDashboard.putNumber("Percent Roller Speed", 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker", beamBreaker.get());
        speed = new Percent(SmartDashboard.getNumber("Percent Roller Speed", 0.0));
        System.out.println(speed.asDouble());
    }

    // public void moveRollers(Percent speed) {
    //     rollers.set(speed.asDouble());  
    // }

    public void moveOuttakeRollers() {
        outtakeRollers.set(speed.asDouble());
    }

    // Needed if were to match different gear ratios
    // public void outtakeRollers() {
    //     Percent speed = new Percent(CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
    //     outtakeRollers.set(speed.asDouble());
    // }

    public void moveIntakeRollers() {
        intakeRollers.set(speed.asDouble());
    }

    // Needed if were to match different gear ratios
    // public void intakeRollers() {
    //     Percent speed = new Percent(CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
    //     intakeRollers.set(speed.asDouble());
    // }

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
        rollers.stopMotor();
    }

    public void stopIntakeRollers() {
        intakeRollers.stopMotor();
    }

    public void stopOuttakeRollers() {
        outtakeRollers.stopMotor();
    }
}