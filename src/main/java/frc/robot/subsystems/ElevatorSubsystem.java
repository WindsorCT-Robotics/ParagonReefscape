package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int MOTOR_CANID = 14;

    private final SparkMax elevMotor;
    private final SparkMaxConfig elevMotorConfig;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder encoder;

    private static final double L1 = 0.0;
    private static final double L2 = 14.0;
    private static final double L3 = 45.0;

    public ElevatorSubsystem() {
        elevMotor = new SparkMax(MOTOR_CANID, MotorType.kBrushless);
        elevMotorConfig = new SparkMaxConfig();

        elevMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        elevMotorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

        closedLoopController = elevMotor.getClosedLoopController();
        encoder = elevMotor.getEncoder();

        elevMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4) // 0.003
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
            .maxVelocity(5600)
            .maxAcceleration(5600)
            .allowedClosedLoopError(0.5)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);    

        elevMotorConfig.idleMode(IdleMode.kBrake);
        elevMotorConfig.inverted(true);
        elevMotor.configure(elevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
        SmartDashboard.putBoolean("Forward Limit Switch", elevMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", elevMotor.getReverseLimitSwitch().isPressed());
    }

    public boolean isAtL3() {
        return (Math.abs(elevMotor.getEncoder().getPosition() - L3) <= 0.5);
    }

    public boolean isAtL2() {
        return (Math.abs(elevMotor.getEncoder().getPosition() - L2) <= 0.5);
    }

    public boolean isAtL1() {
        return (Math.abs(elevMotor.getEncoder().getPosition() - L1) <= 0.5);
    }

    public void setToL3(){
        closedLoopController.setReference(L3, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);    
    }

    public void setToL2(){
        closedLoopController.setReference(L2, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);    
    }

    public void setToL1(){
        closedLoopController.setReference(L1, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);    
    }

    public void stopMotor() {
        elevMotor.stopMotor();
    }
}