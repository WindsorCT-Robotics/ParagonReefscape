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
    private static final int ROLLER_LEFT_MOTOR_CANID = 15;
    private static final int ROLLER_RIGHT_MOTOR_CANID = 16;
    private static final int BEAM_BREAKER_PIN = 0;

    private SparkMax rollerLeft;
    private SparkMax rollerRight;
    private RelativeEncoder rollerEncoder;

    private SparkMaxConfig rollerLeftConfig;
    private SparkMaxConfig rollerRightConfig;

    private final Percent speed;
    private final Percent speedFast;
    private final Percent speedSlow;

    private final DigitalInput beamBreaker;

    public CarriageSubsystem() {
        speed = new Percent(0.25);
        speedFast = new Percent(0.27);
        speedSlow = new Percent(0.07);
        rollerLeft = new SparkMax(ROLLER_LEFT_MOTOR_CANID, MotorType.kBrushless);
        rollerRight = new SparkMax(ROLLER_RIGHT_MOTOR_CANID, MotorType.kBrushless);
        rollerLeftConfig = new SparkMaxConfig();
        rollerRightConfig = new SparkMaxConfig();

        rollerLeftConfig.inverted(true);
        rollerRightConfig.inverted(false);

        rollerLeftConfig.idleMode(IdleMode.kBrake);
        rollerRightConfig.idleMode(IdleMode.kBrake);

        rollerLeft.configure(rollerLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerRight.configure(rollerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        beamBreaker = new DigitalInput(BEAM_BREAKER_PIN);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Beam Breaker", beamBreaker.get());
        
    }

    public void moveRollers() {
        rollerLeft.set(speed.asDouble());
        rollerRight.set(speed.asDouble());
    }

    public void manualMoveRollers(Percent speed) {
        rollerLeft.set(speed.asDouble());
        rollerRight.set(speed.asDouble());
    }

    public void moveRollersLeft() {
        rollerLeft.set(speedFast.asDouble());
        rollerRight.set(speedSlow.asDouble());
    }

    public void moveRollersRight() {
        rollerLeft.set(speedSlow.asDouble());
        rollerRight.set(speedFast.asDouble());
    }

    public boolean isBeamBroken() {
        return !beamBreaker.get();
    }

    public void resetRollerEncoder() {
        rollerEncoder.setPosition(0.0);  
    }
    
    public void stopRollers() {
        rollerLeft.stopMotor();
        rollerRight.stopMotor();
    }

    public Percent getSpeedSlow() {
        return speedSlow;
    }

    public Percent getSpeedFast() {
        return speedFast;
    }
}