package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

public class SimCommandSwerveDrivetrain extends CommandSwerveDrivetrain {
    private static final Time SIM_LOOP_PERIOD = Millisecond.of(5);
    private static final Mass ROBOT_WEIGHT = Kilogram.of(53.977);
    private static final Distance BUMPER_LENGTH = Meters.of(0.864);
    private static final double WHEEL_COEFFICIENT_OF_FRICTION = 1;
    private static final int DRIVE_MOTOR_COUNT = 1;
    private static final int STEER_MOTOR_COUNT = 1;

    public final MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = 
        new MapleSimSwerveDrivetrain(
                SIM_LOOP_PERIOD,
                ROBOT_WEIGHT,
                BUMPER_LENGTH,
                BUMPER_LENGTH,
                DCMotor.getKrakenX60Foc(DRIVE_MOTOR_COUNT), // drive motor type
                DCMotor.getKrakenX60Foc(STEER_MOTOR_COUNT), // steer motor type
                WHEEL_COEFFICIENT_OF_FRICTION,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);

    private Notifier simNotifier;

    public SimCommandSwerveDrivetrain(String name,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(name, 
            drivetrainConstants, 
            modules);
        startSimThread();
    }

    public SimCommandSwerveDrivetrain(String name,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(name, 
            drivetrainConstants, 
            odometryUpdateFrequency, 
            modules);
        startSimThread();
    }

    public SimCommandSwerveDrivetrain(String name,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(name, 
            drivetrainConstants, 
            odometryUpdateFrequency, 
            odometryStandardDeviation, 
            visionStandardDeviation, 
            modules);
            
        startSimThread();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    private void startSimThread() {
        simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simNotifier.startPeriodic(SIM_LOOP_PERIOD.in(Seconds));
    }
    
    @Override
    public void close() {
        super.close();
        simNotifier.close();
    }
}
