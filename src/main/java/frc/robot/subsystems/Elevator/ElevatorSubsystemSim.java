package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystemSim extends SubsystemBase {

    private final static SparkMax elevMotor = ElevatorSubsystem.getElevatorMotor();
    private final static SparkMaxSim elevMotorSim = new SparkMaxSim(elevMotor, DCMotor.getKrakenX60(1));
    
    private final SparkMaxConfig elevMotorConfig;

    // private static final double L1 = 0;
    // private static final double L2 = 8.5;
    // private static final double L2_5 = 21;
    // private static final double L3 = 27;

    private final double gravityVoltage = 0.4;

    private final LoggedMechanism2d visualizedElevator;
    private final LoggedMechanismRoot2d elevatorRoot;
    private final LoggedMechanismLigament2d elevatorLigament;

    private final TrapezoidProfile.Constraints elevatorConstraints = new Constraints(10, 4);

    // 60 of position is equal to 1 meter
    private TrapezoidProfile.State L1 = new TrapezoidProfile.State(0, 0.0);
    private TrapezoidProfile.State L2 = new TrapezoidProfile.State(9.906, 0.0);
    private TrapezoidProfile.State L2_5 = new TrapezoidProfile.State(28, 0.0);
    private TrapezoidProfile.State L3 = new TrapezoidProfile.State(34.4805, 0.0);

    private TrapezoidProfile.State current = new TrapezoidProfile.State(0.0, 0.0);

    private TrapezoidProfile.State goal;
    private double velocity = 0.0;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State next = new TrapezoidProfile.State(0.0, 0.0);
    private final PIDController pid;

    public ElevatorSubsystemSim() {
        pid = new PIDController(1, 0, 0);
        goal = L1;
        profile = new TrapezoidProfile(elevatorConstraints);

        visualizedElevator = new LoggedMechanism2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        elevatorRoot = visualizedElevator.getRoot("Elevator", Units.inchesToMeters(0), Units.inchesToMeters(0));
        elevatorLigament = elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", Units.inchesToMeters(50), 90));

        elevMotorConfig = new SparkMaxConfig();

        elevMotorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        elevMotorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

        elevMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.3) // 0.003
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
            .allowedClosedLoopError(0.1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);    

        elevMotorConfig.idleMode(IdleMode.kBrake);
        elevMotorConfig.inverted(true);
        // elevMotorConfig.smartCurrentLimit(60);
        elevMotor.configure(elevMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    @Override
    public void periodic() {
        if (profile != null && current.position != goal.position) {
            next = profile.calculate(0.2, current, goal);
            velocity = pid.calculate(current.position, next.position);
        } else{
            velocity = 0;
        }

        elevMotorSim.iterate(velocity, 12, 1);
        Logger.recordOutput("Height", elevMotorSim.getPosition());

        // Logger.recordOutput("Mechanism", visualizedElevator);
        current = next;
    }

    public void holdPosition() {
        elevMotor.setVoltage(gravityVoltage);
    }

    public boolean isAtL3() {
        return (Math.abs(elevMotorSim.getPosition() - L3.position) <= 0.1);
    }

    @AutoLogOutput(key = "Elevator/L2.5")
    public boolean isAtL2_5() {
        return (Math.abs(elevMotorSim.getPosition() - L2_5.position) <= 0.1);
    }

    @AutoLogOutput(key = "Elevator/L2")
    public boolean isAtL2() {
        return (Math.abs(elevMotorSim.getPosition() - L2.position) <= 0.1);
    }

    @AutoLogOutput(key = "Elevator/L1")
    public boolean isAtL1() {
        return (Math.abs(elevMotorSim.getPosition() - L1.position) <= 0.1);
    }

    public void setToL3() {
        goal = L3;
        profile = new TrapezoidProfile(elevatorConstraints);
    }

    public void setToL2_5() {
        goal = L2_5;
        profile = new TrapezoidProfile(elevatorConstraints);
    }

    public void setToL2() {
        goal = L2;
        profile = new TrapezoidProfile(elevatorConstraints);
    }

    public void setToL1() {
        goal = L1;
        profile = new TrapezoidProfile(elevatorConstraints);
    }

    public void moveMotor() {
        elevMotor.set(-0.04);
    }

    @AutoLogOutput(key = "Elevator/ReverseLimit")
    public boolean getLowerLimit() {
        return elevMotor.getReverseLimitSwitch().isPressed();
    }

    @AutoLogOutput(key = "Elevator/ForwardLimit")
    public double getElevEncoderPosition() {
        return elevMotor.getEncoder().getPosition();
    }

    public void resetRelativeEncoder() {
        elevMotor.getEncoder().setPosition(0);
    }

    public void stopMotor() {
        elevMotor.stopMotor();
    }

    public static double getHeight() {
        return elevMotorSim.getPosition();
    }
}