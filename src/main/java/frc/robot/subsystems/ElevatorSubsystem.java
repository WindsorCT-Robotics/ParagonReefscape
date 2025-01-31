package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int MOTOR_CANID = 14;
    private static final int UPPER_LIMIT_PIN = 0;
    private static final int LOWER_LIMIT_PIN = 1;

    private final double TARGET_UPPER = 0.0; //Temporary value
    private final double TARGET_LOWER = 0.0;

    private final SparkMax elevMotor;
    private final SparkMaxConfig elevMotorConfig;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;


    private final DigitalInput upperLimit;
    private final DigitalInput lowerLimit;

    public ElevatorSubsystem() {
        elevMotor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
        elevMotorConfig = new SparkMaxConfig();

        closedLoopController = elevMotor.getClosedLoopController();
        encoder = elevMotor.getEncoder();

        elevMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            // Set PID values for velocity control in slot 1
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        
        elevMotorConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(1000)
            .maxAcceleration(1000)
            .allowedClosedLoopError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);    

        elevMotorConfig.inverted(true);
        elevMotorConfig.idleMode(IdleMode.kBrake);
        elevMotor.configure(elevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        upperLimit = new DigitalInput(UPPER_LIMIT_PIN);
        lowerLimit = new DigitalInput(LOWER_LIMIT_PIN);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Upper Limit Switch", upperLimit.get());
        SmartDashboard.putBoolean("Lower Limit Switch", lowerLimit.get());
    }


    public void setToTop(){
        closedLoopController.setReference(TARGET_UPPER, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);    
    }


    public void setToBottom(){
        closedLoopController.setReference(TARGET_LOWER, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);    
    }

    public boolean isAtTop() {
        return upperLimit.get();
    }

    public boolean isAtBottom() {
        return lowerLimit.get();
    }

    public void motorStop() {
        elevMotor.stopMotor();
    }
}