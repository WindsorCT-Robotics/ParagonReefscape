package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Units.Percent;
import frc.robot.Units.Rotations;
import frc.robot.Units.RotationsPerMinute;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
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
    private SparkMax moveouttakeRollers;
    private SparkMax moveIntakeRollers;
    private SparkMaxConfig rollersConfig;
    private final DigitalInput beamBreaker;
    private RelativeEncoder rollerEncoder;
    private RelativeEncoder outtakeEncoder;
    private RelativeEncoder intakeEncoder;

    private static final int MOTOR_MAX_RPM = 11000;
    private static final double MOTOR_GEAR_RATIO = (double) 1 / 16; // Gear ratio is 4:1, 4:1

    public static RotationsPerMinute targetRPM = new RotationsPerMinute(300, 1);

    private static final double ROLLERS_CIRCUMFERENCE = Math.PI * 1.67; // Outer diameter is 1.67"

    public CarriageSubsystem() {
        moveouttakeRollers = new SparkMax(OUTTAKE_MOTOR_CANID, MotorType.kBrushless);
        moveIntakeRollers = new SparkMax(INTAKE_MOTOR_CANID, MotorType.kBrushless);
        rollersConfig = new SparkMaxConfig();

        rollersConfig.inverted(true);
        rollersConfig.idleMode(IdleMode.kBrake);

        rollers.configure(rollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);
        rollerEncoder = rollers.getEncoder();
        outtakeEncoder = moveouttakeRollers.getEncoder();
        intakeEncoder = moveouttakeRollers.getEncoder();

        SmartDashboard.putNumber("Outtake Percent Speed", CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker ", beamBreaker.get());
    }

    // public void moveRollers(Percent speed) {
    //     rollers.set(speed.asDouble());
    // }

    public void moveouttakeRollers(Percent percent) {
        Percent speed = new Percent(CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
        moveouttakeRollers.set(speed.asDouble());
    }

    public void outtakeRollers(Percent speed) {
        moveouttakeRollers.set(speed.asDouble());
    }

    public void moveIntakeRollers(Percent percent) {
        Percent speed = new Percent(CarriageSubsystem.targetRPM.asDouble() / (MOTOR_MAX_RPM * MOTOR_GEAR_RATIO));
        moveIntakeRollers.set(speed.asDouble());
    }

    public void intakeRollers(Percent speed) {
        moveIntakeRollers.set(speed.asDouble());
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
        rollerEncoder.setPosition(0.0);
    }
    
    public void stop() {
    // rollers.stopMotor();
    moveIntakeRollers.stopMotor();
    moveouttakeRollers.stopMotor();
    }
}