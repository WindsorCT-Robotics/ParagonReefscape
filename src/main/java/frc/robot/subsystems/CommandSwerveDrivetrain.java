package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.apriltag.AprilTagLocation;
import frc.robot.apriltag.ReefscapeAprilTagFieldLayoutMapper;
import frc.robot.apriltag.ReefscapeApriltag;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.hardware.IDistanceSensor;
import frc.robot.result.Failure;
import frc.robot.result.Result;
import frc.robot.result.Success;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Sendable {
    private static final LinearVelocity MAX_LINEAR_VELOCITY = TunerConstants.kSpeedAt12Volts;
    private static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(1.0);

    private static final PIDConstants TRANSLATION_PID = new PIDConstants(3.0, 0.0, 0.0);
    private static final PIDConstants ROTATION_PID = new PIDConstants(7.0, 0.0, 0.0);

    public sealed interface AprilTagSearchError permits NoAprilTagsFound, AllianceUnknown { }
    public record NoAprilTagsFound() implements AprilTagSearchError { }
    public record AllianceUnknown() implements AprilTagSearchError { }

    private final SwerveRequest.ApplyRobotSpeeds pathDriveController = new SwerveRequest.ApplyRobotSpeeds();
    private static final LinearVelocity TOF_SPEED = MetersPerSecond.of(0.6);

    private RobotConfig config;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private static final Distance THRESHOLD_DISTANCE = Millimeters.of(400);
    private static final Time DEBOUNCE_TIME = Milliseconds.of(50);
    private final IDistanceSensor leftTofSensor;
    private final IDistanceSensor rightTofSensor;

    private enum BranchAlignment {
        ALIGN_LEFT,
        ALIGN_RIGHT
    }

    private static final Distance BRANCH_DISTANCE = Meters.of(0.1651);
    private static final Distance DEFAULT_CONTROL_DISTANCE = Meters.of(0.5);

    public final Trigger isLeftReefAligned = new Trigger(() -> isBranchAligned(BranchAlignment.ALIGN_LEFT))
            .debounce(DEBOUNCE_TIME.in(Seconds));
    public final Trigger isRightReefAligned = new Trigger(() -> isBranchAligned(BranchAlignment.ALIGN_RIGHT))
            .debounce(DEBOUNCE_TIME.in(Seconds));

    private static final LinearVelocity LINEAR_VELOCITY_CONSTRAINT = MetersPerSecond.of(0.0);
    private static final LinearAcceleration LINEAR_ACCELERATION_CONSTRAINT = MetersPerSecondPerSecond.of(0.0);
    private static final AngularVelocity ANGULAR_VELOCITY_CONSTRAINT = DegreesPerSecond.of(0.0);
    private static final AngularAcceleration ANGULAR_ACCELERATION_CONSTRAINT = DegreesPerSecondPerSecond.of(0.0);
    private static final Voltage NOMIAL_VOLTAGE = Volts.of(12.0); 
    // TODO: Once robot refactoring is completed, experiment with values.

    private static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
        LINEAR_VELOCITY_CONSTRAINT, 
        LINEAR_ACCELERATION_CONSTRAINT, 
        ANGULAR_VELOCITY_CONSTRAINT, 
        ANGULAR_ACCELERATION_CONSTRAINT, 
        NOMIAL_VOLTAGE, 
        false);

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine sysIdRoutineToApply = sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param name                Name of the subsystem
     * @param leftTofSensor       Distance sensor for calibrating alignment to the
     *                            left branch on a reef.
     * @param rightTofSensor      Distance sensor for calibrating alignment to the
     *                            right branch on a reef.
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(String name,
            IDistanceSensor leftTofSensor,
            IDistanceSensor rightTofSensor,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.leftTofSensor = leftTofSensor;
        this.rightTofSensor = rightTofSensor;
        SendableRegistry.addLW(this, name, name);
        CommandScheduler.getInstance().registerSubsystem(this);
        configureAutobuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param name                    Name of the subsystem
     * @param leftTofSensor           Distance sensor for calibrating alignment to
     *                                the left branch on a reef.
     * @param rightTofSensor          Distance sensor for calibrating alignment to
     *                                the right branch on a reef.
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(String name,
            IDistanceSensor leftTofSensor,
            IDistanceSensor rightTofSensor,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.leftTofSensor = leftTofSensor;
        this.rightTofSensor = rightTofSensor;
        SendableRegistry.addLW(this, name, name);
        CommandScheduler.getInstance().registerSubsystem(this);
        configureAutobuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param name                      Name of the subsystem
     * @param leftTofSensor             Distance sensor for calibrating alignment to
     *                                  the left branch on a reef.
     * @param rightTofSensor            Distance sensor for calibrating alignment to
     *                                  the right branch on a reef.
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(String name,
            IDistanceSensor leftTofSensor,
            IDistanceSensor rightTofSensor,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        this.leftTofSensor = leftTofSensor;
        this.rightTofSensor = rightTofSensor;
        SendableRegistry.addLW(this, name, name);
        CommandScheduler.getInstance().registerSubsystem(this);
        configureAutobuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);

        builder.addStringProperty(
                ".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                null);

        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);

        builder.addStringProperty(
                ".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                null);

        builder.addDoubleProperty("Yaw", () -> getPigeon2().getYaw().getValueAsDouble(), null);
        builder.addDoubleProperty("Drivetrain X", () -> getState().Pose.getTranslation().getX(), null);
        builder.addDoubleProperty("Drivetrain Y", () -> getState().Pose.getTranslation().getY(), null);
        builder.addDoubleProperty("Drivetrain Current Direction", () -> getState().Pose.getRotation().getDegrees(),
                null);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void configureAutobuilder() {
        // Code for pathplanning. Can be found at this link:
        // https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            DriverStation.reportError("Failed to configure AutoBuilder: " + e.getMessage(), e.getStackTrace());
        }

        try {

            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            pathDriveController.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private boolean isBranchAligned(BranchAlignment alignment) {
        Distance distance = (alignment == BranchAlignment.ALIGN_LEFT)
                ? leftTofSensor.getDistance()
                : rightTofSensor.getDistance();

        return distance.gte(THRESHOLD_DISTANCE);
    }

    private Command stop() {
        return runOnce(() -> setControl(new RobotCentric()));
    }

    private enum RelativeReference {
        ROBOT_RELATIVE,
        FIELD_RELATIVE
    }

    public Command move(Supplier<LinearVelocity> velocityX, Supplier<LinearVelocity> velocityY,
            Supplier<AngularVelocity> rotationalRate, RelativeReference relativeReference) {
        switch (relativeReference) {
            case ROBOT_RELATIVE:
                return robotRelativeSwerveRequest(velocityX, velocityY, rotationalRate);
            case FIELD_RELATIVE:
                return fieldRelativeSwerveRequest(velocityX, velocityY, rotationalRate);
            default:
                return robotRelativeSwerveRequest(() -> MetersPerSecond.zero(), () -> MetersPerSecond.zero(), () -> DegreesPerSecond.zero());
        }
    }

    private Command robotRelativeSwerveRequest(
        Supplier<LinearVelocity> velocityX, Supplier<LinearVelocity> velocityY,
        Supplier<AngularVelocity> rotationalRate) {
        return run(() -> setControl(
                new RobotCentric()
                        .withVelocityX(velocityX.get())
                        .withVelocityY(velocityY.get())
                        .withRotationalRate(rotationalRate.get())));
    }

    private Command fieldRelativeSwerveRequest(
        Supplier<LinearVelocity> velocityX, Supplier<LinearVelocity> velocityY,
        Supplier<AngularVelocity> rotationalRate) {
        return run(() -> setControl(
                new FieldCentric()
                        .withVelocityX(velocityX.get())
                        .withVelocityY(velocityY.get())
                        .withRotationalRate(rotationalRate.get())));
    }

    public Command moveWithPercentages(Supplier<Dimensionless> percentX, Supplier<Dimensionless> percentY,
            Supplier<Dimensionless> percentRotationalRate, RelativeReference relativeReference) {
        return move(
                () -> calculateLinearVelocityFromPercentage(percentX),
                () -> calculateLinearVelocityFromPercentage(percentY),
                () -> calculateAngularVelocityFromPercentage(percentRotationalRate),
                relativeReference);
    }

    private LinearVelocity calculateLinearVelocityFromPercentage(Supplier<Dimensionless> percent) {
        return MetersPerSecond.of(percent.get().times(MAX_LINEAR_VELOCITY.in(MetersPerSecond)).in(Value));
    }

    private AngularVelocity calculateAngularVelocityFromPercentage(Supplier<Dimensionless> percent) {
        return RadiansPerSecond.of(percent.get().times(MAX_ANGULAR_VELOCITY.in(RadiansPerSecond)).in(Value));
    }

    public Command pathAndAlignToClosestSideBranch(BranchAlignment branchAlignment) {
        return new SequentialCommandGroup(
            pathToClosestSideBranch(branchAlignment, DEFAULT_PATH_CONSTRAINTS),
            alignToBranch(branchAlignment));
    }

    public Command alignToBranch(BranchAlignment alignment) {
        return runEnd(
                () -> move(MetersPerSecond::zero,
                        () -> (alignment == BranchAlignment.ALIGN_LEFT) ? TOF_SPEED.times(-1) : TOF_SPEED,
                        RadiansPerSecond::zero,
                        RelativeReference.ROBOT_RELATIVE),
                this::stop)
                .until((alignment == BranchAlignment.ALIGN_LEFT) ? isLeftReefAligned : isRightReefAligned);
    }

    private Result<ReefscapeApriltag, AprilTagSearchError> findClosestTag() {
        ReefscapeAprilTagFieldLayoutMapper mapper = new ReefscapeAprilTagFieldLayoutMapper();
        ReefscapeApriltag closestTag;
        List<ReefscapeApriltag> tags = mapper.getTags();
        Alliance alliance;
        Pose2d robotPosition = getState().Pose;

        if (tags.isEmpty()) {
            return new Failure<>(new NoAprilTagsFound());
        }

        if (DriverStation.getAlliance().isEmpty()) {
            return new Failure<>(new AllianceUnknown());
        }

        alliance = DriverStation.getAlliance().orElseThrow();

        closestTag = tags.stream()
            .filter(tag -> tag.alliance == alliance && tag.location == AprilTagLocation.REEF)
            .min((tag1, tag2) -> pointToPointDistance(new Pose3d(robotPosition), tag1.pose).compareTo(pointToPointDistance(new Pose3d(robotPosition), tag2.pose)))
            .get();

        return new Success<>(closestTag);
    }
    
    private Distance pointToPointDistance(Pose3d pose1, Pose3d pose2) {
        Distance differenceX = Meters.of(pose1.getX() - pose2.getX());
        Distance differenceY = Meters.of(pose1.getY() - pose2.getY());

        return pythagoreanDistance(differenceX, differenceY);
    }

    private Distance pythagoreanDistance(
        Distance x, 
        Distance y) {
        Distance squaredSum = Meters.of(Math.pow(x.in(Meters), 2) + Math.pow(y.in(Meters), 2));
        return Meters.of(Math.sqrt(squaredSum.in(Meters)));
    }

    private Pose3d poseToBranch(Pose3d pose, BranchAlignment branchSide) {
        switch (branchSide) {
            case ALIGN_LEFT:
                pose = translateTo(pose, Degrees.of(90), Degrees.zero(), BRANCH_DISTANCE.times(-1));
                break;
            case ALIGN_RIGHT:
                pose = translateTo(pose, Degrees.of(90), Degrees.zero(), BRANCH_DISTANCE);
                break;
        }

        return pose;
    }

    private Pose3d translateTo(Pose3d pose, Angle yaw, Angle pitch, Rotation3d rotation, Distance distance) {
        Distance translationDistanceX = distance.times(Math.sin(yaw.in(Radians))).times(Math.cos(pitch.in(Radians)));
        Distance translationDistanceY = distance.times(Math.sin(pitch.in(Radians)));
        Distance translationDistanceZ = distance.times(Math.cos(yaw.in(Radians))).times(Math.cos(pitch.in(Radians)));

        return pose.plus(new Transform3d(translationDistanceX, translationDistanceY, translationDistanceZ, rotation));
    }

    private Pose3d translateTo(Pose3d pose, Angle yaw, Angle pitch, Distance distance) {
        return translateTo(pose, yaw, pitch, pose.getRotation(), distance);
    }

    private PathPlannerPath createPathToTag(
        Pose3d tag, 
        BranchAlignment branchAlignment,
        PathConstraints pathConstraints
    ) {
        SwerveDriveState robotCurrentState = getState();
        Pigeon2 gyro = getPigeon2();

        ChassisSpeeds robotRelativeChassisSpeeds = robotCurrentState.Speeds;
        ChassisSpeeds fieldRelativeChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeChassisSpeeds, gyro.getRotation2d());

        Translation2d robotWaypointAnchor = robotCurrentState.Pose.getTranslation();

        LinearVelocity robotVelocity = MetersPerSecond.of(
            Math.sqrt(
                Math.pow(robotRelativeChassisSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeChassisSpeeds.vyMetersPerSecond, 2)
        ));
        Angle directionOfVelocity = Radians.of(
            Math.atan2(fieldRelativeChassisSpeeds.vyMetersPerSecond, fieldRelativeChassisSpeeds.vxMetersPerSecond));

        Translation2d prevControlRobot = new Translation2d();
        Translation2d nextControlRobot = new Translation2d(DEFAULT_CONTROL_DISTANCE.in(Meters), new Rotation2d(directionOfVelocity));
        
        Translation2d branchPose = poseToBranch(tag, branchAlignment).getTranslation().toTranslation2d();
        Angle invertedBranchYawAngle = Degrees.of(branchPose.getAngle().getDegrees() + 180 % 360);
        Translation2d prevControlBranch = new Translation2d(DEFAULT_CONTROL_DISTANCE.in(Meters), new Rotation2d(invertedBranchYawAngle.in(Degrees)));
        Translation2d nextControlBranch = new Translation2d();

        Waypoint branchWaypoint = new Waypoint(prevControlBranch, branchPose, nextControlBranch);
        Waypoint robotPositionWaypoint = new Waypoint(prevControlRobot, robotWaypointAnchor, nextControlRobot);

        List<Waypoint> waypoints = List.of(robotPositionWaypoint, branchWaypoint);
        IdealStartingState idealStartingState = new IdealStartingState(robotVelocity, new Rotation2d(gyro.getYaw().getValue()));
        GoalEndState goalEndState = new GoalEndState(MetersPerSecond.zero(), new Rotation2d(invertedBranchYawAngle));

        return new PathPlannerPath(waypoints, pathConstraints, idealStartingState, goalEndState);
    }

    private Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    private Command pathFindThenFollowPath(PathPlannerPath path, PathConstraints pathConstraints) {
        return AutoBuilder.pathfindThenFollowPath(path, pathConstraints);
    }

    public Command pathToTag(
        Pose3d tag, 
        BranchAlignment branchAlignment,
        PathConstraints pathConstraints
    ) {
        PathPlannerPath path = createPathToTag(tag, branchAlignment, pathConstraints);

        return followPath(path);
    }

    public Command pathFindThenPathToTag(
        Pose3d tag, 
        BranchAlignment branchAlignment,
        PathConstraints pathConstraints,
        GoalEndState goalEndState
    ) {
        PathPlannerPath path = createPathToTag(tag, branchAlignment, pathConstraints);

        return pathFindThenFollowPath(path, pathConstraints);
    }

    public Command pathToClosestSideBranch(
        BranchAlignment branchAlignment,
        PathConstraints pathConstraints
    ) {
        Pose3d branchPose = translateTo(findClosestTag().getValue().pose, Degrees.of(90.0), Degrees.zero(), BRANCH_DISTANCE);
        return pathToTag(branchPose, branchAlignment, pathConstraints);
    }

    public Command addVisionMeasurementToRobotPosition(Supplier<PoseEstimate> positionEstimate) {
        return run(() -> 
            this.addVisionMeasurement(positionEstimate.get().pose, Utils.fpgaToCurrentTime(positionEstimate.get().timestampSeconds)))
            .asProxy();
    }

    public RobotConfig getPathConfig() {
        return config;
    }

    public LinearVelocity getMaxVelocity() {
        return MAX_LINEAR_VELOCITY;
    }

    public AngularVelocity getMaxAngularRate() {
        return MAX_ANGULAR_VELOCITY;
    }

    public PIDConstants getTranslationPID() {
        return TRANSLATION_PID;
    }

    public PIDConstants getRotationPID() {
        return ROTATION_PID;
    }

    public SwerveRequest.ApplyRobotSpeeds getPathDriveController() {
        return pathDriveController;
    }
}