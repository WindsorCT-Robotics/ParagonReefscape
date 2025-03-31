package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import frc.lib.Limelight.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import com.pathplanner.lib.config.PIDConstants;

import dev.doglog.DogLog;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double aprilTagID;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final TimeOfFlight lTOFSensor;
    private final TimeOfFlight rTOFSensor;

    private final double TOFSensorThreshold = 400;

    private static final int L_TOF_CANID = 19;
    private static final int R_TOF_CANID = 18;

    // For On the fly pathing to april tags
    Pose2d[][] aprilTagPoses = new Pose2d[23][2];
    double[][] aprilTagPositions = new double[23][3];
    private static final Set<Integer> usedAprilTags = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    private final double[] reefDirections = {0, 60, 120, 180, -60, -120};

    private final double redAdjustX = 8.569452;
    private final double redAdjustY = 0.0;
    private final double preReefAdjust = -0.3;
    private final double preCoralStationAdjust = 0.3;

    private final double leftAngle = 90.0;
    private final double rightAngle = -90.0;

    private final double branchOffset = 0.1651;
    // end on the fly

    private Field2d field = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(getKinematics(), getPigeon2().getRotation2d(), , null);

    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }

        getPigeon2().getConfigurator().setYaw(0.0);

        configureAutoBuilder();
        SmartDashboard.putData(field);

        
        rTOFSensor = new TimeOfFlight(R_TOF_CANID);
        lTOFSensor = new TimeOfFlight(L_TOF_CANID);
        configTOFSensors();

        createAlignmentConstants();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }

        getPigeon2().getConfigurator().setYaw(0.0);

        configureAutoBuilder();
        SmartDashboard.putData(field);

        
        rTOFSensor = new TimeOfFlight(R_TOF_CANID);
        lTOFSensor = new TimeOfFlight(L_TOF_CANID);
        configTOFSensors();

        createAlignmentConstants();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }

        getPigeon2().getConfigurator().setYaw(0.0);

        configureAutoBuilder();
        SmartDashboard.putData(field);

        
        rTOFSensor = new TimeOfFlight(R_TOF_CANID);
        lTOFSensor = new TimeOfFlight(L_TOF_CANID);
        configTOFSensors();

        createAlignmentConstants();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @SuppressWarnings("unused")
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    private List<Waypoint> trajectory(boolean isCoralStation, double aprilTagID, String direction, Rotation2d orientation) {
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        
        // Checks if the id that is being used is an id that is allowed to be used for positioning
        if (!usedAprilTags.contains((int) aprilTagID)) {
            return null;
        }

        double[] prePose = {aprilTagPoses[(int) aprilTagID][0].getX(), aprilTagPoses[(int) aprilTagID][0].getY()};
        double[] pose = {aprilTagPoses[(int) aprilTagID][1].getX(), aprilTagPoses[(int) aprilTagID][1].getY()};

        // Reef Alignment
        if (direction.equalsIgnoreCase("left")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(calculateDirectionalTranslation(prePose[0], branchOffset, orientation.getDegrees() + leftAngle, "x"), calculateDirectionalTranslation(prePose[1], branchOffset, orientation.getDegrees() + leftAngle, "y"), orientation), 
                new Pose2d(calculateDirectionalTranslation(pose[0], branchOffset, orientation.getDegrees() + leftAngle, "x"), calculateDirectionalTranslation(pose[1], branchOffset, orientation.getDegrees() + leftAngle, "y"), orientation));
        }

        if (direction.equalsIgnoreCase("center")) {
            if (isCoralStation == true) {
                // Blue
                if (this.getState().Pose.getY() < 4.025) {
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        aprilTagID = 12.0;
                    } else {
                        aprilTagID = 1.0;
                    }
                }
                
                if (this.getState().Pose.getY() > 4.025) {
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        aprilTagID = 13.0;
                    } else {
                        aprilTagID = 2.0;
                    }
                }

                orientation = Rotation2d.fromDegrees(aprilTagPoses[(int) aprilTagID][0].getRotation().getDegrees());
            }
            
            waypoints = PathPlannerPath.waypointsFromPoses(aprilTagPoses[(int) aprilTagID][0], aprilTagPoses[(int) aprilTagID][1]);
        }

        if (direction.equalsIgnoreCase("right")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(calculateDirectionalTranslation(prePose[0], branchOffset, orientation.getDegrees() + rightAngle, "x"), calculateDirectionalTranslation(prePose[1], branchOffset, orientation.getDegrees() + rightAngle, "y"), orientation), 
                new Pose2d(calculateDirectionalTranslation(pose[0], branchOffset, orientation.getDegrees() + rightAngle, "x"), calculateDirectionalTranslation(pose[1], branchOffset, orientation.getDegrees() + rightAngle, "y"), orientation));
        }
        return waypoints;
    }

    public double calculateDirectionalTranslation(double currentPose, double distance, double angle, String conditional) {
        if (distance == 0) {
            return currentPose;
        }
        
        if (conditional.equalsIgnoreCase("x")) {
            return currentPose + distance * Math.cos(Math.toRadians(angle));
        }

        if (conditional.equalsIgnoreCase("y")) {
            return currentPose + distance * Math.sin(Math.toRadians(angle));
        }

        System.out.println("Didn't pass any statements");
        return currentPose;
    }

    public void createOffsets(double reefOffset, double coralStationOffset) {
        for (int row = 0; row < aprilTagPositions.length; row++) {
            for (int col = 0; col < aprilTagPositions[row].length - 1; col++) {
                if (col == 0) {
                    if (Set.of(1, 2, 12, 13).contains(row)) {
                        aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], coralStationOffset, aprilTagPositions[row][2], "x"); // X coordinate
                    } else {
                        aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], reefOffset, aprilTagPositions[row][2], "x"); // X coordinate
                    }
                } else {
                    if (Set.of(1, 2, 12, 13).contains(row)) {
                        aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], coralStationOffset, aprilTagPositions[row][2], "y"); // X coordinate
                    } else {
                        aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], reefOffset, aprilTagPositions[row][2], "y"); // Y coordinate
                    }
                }
            }
        }
    }

    public double createPreAdjustments(double distance, int row, int col) {
        if (col == 0) {
            return calculateDirectionalTranslation(aprilTagPositions[row][col], distance, aprilTagPositions[row][2], "x"); // X coordinate
        } else {
            return calculateDirectionalTranslation(aprilTagPositions[row][col], distance, aprilTagPositions[row][2], "y"); // Y coordinate
        }
    }

    public Command pathToAlignGenerator(Limelight limelight, boolean isCoralStation, String direction) {
        System.out.println("Begin generating path");

        List<Waypoint> waypoints;
        Rotation2d orientation;
        aprilTagID = LimelightHelpers.getFiducialID(limelight.getLimelightName());
        System.out.println("Current apriltag is " + aprilTagID);
        
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        if (usedAprilTags.contains((int) aprilTagID)) {
            orientation = Rotation2d.fromDegrees(aprilTagPoses[(int) aprilTagID][0].getRotation().getDegrees());
            waypoints = trajectory(isCoralStation, aprilTagID, direction, orientation);
        } else {
            System.out.println("No valid apriltag");
            return Commands.none();
        }

        if (waypoints == null) {
            System.out.println("Waypoints is null");
            return Commands.none();
        }

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, orientation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        try {
            System.out.println("Path made and executing");
            return new FollowPathCommand(path, () -> getState().Pose, () -> getState().Speeds, (speeds, feedforwards) -> 
            setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
            new PPHolonomicDriveController(
                // PID constants for translation
                new PIDConstants(3, 0, 0),
                // PID constants for rotation
                new PIDConstants(7, 0, 0)
            ), RobotConfig.fromGUISettings(), () -> false, this);
        } catch (IOException | ParseException ex) {
            System.out.println("No Command due to try-catch");
            return Commands.none();
        }
    }

    public Command pathToAlign(Limelight limelight, boolean isCoralStation, String direction) {
        System.out.println("Calling Deferred Command");
        return new DeferredCommand(() -> pathToAlignGenerator(limelight, isCoralStation, direction), Set.of(this, limelight));
    }

    public Command setOrientation(CommandXboxController driverController, Limelight limelight, boolean isCoralStation) {
        return new Command() {
            double orientation;
            
            @Override
            public void initialize() {
                orientation = 0;
                SmartDashboard.putBoolean("Angle Align", true);
            }

            @Override
            public void execute() {
                if (!isCoralStation) {
                    if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
                        if (getState().Pose.getY() >= 4.4959 || getState().Pose.getY() <= 3.5561) {
                            if (getState().Pose.getY() > 4.026) {
                                orientation = -54;
                            } else {
                                orientation = 54;
                            }
                        } else {
                            orientation = 0;
                        }
                    } else {
                        if (getState().Pose.getY() >= 4.4959 || getState().Pose.getY() <= 3.5561) {
                            if (getState().Pose.getY() > 4.026) {
                                orientation = 54;
                            } else {
                                orientation = -54;
                            }
                        } else {
                            orientation = 0;
                        }
                    }

                    if (Math.abs(getState().Pose.getRotation().getDegrees() - orientation) >= 1) {
                        setControl(new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(7, 0, 0).withTargetDirection(Rotation2d.fromDegrees(orientation)).withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))); // Drive left with negative X (left)
                    } else {
                        setControl(new SwerveRequest.FieldCentric().withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))); // Drive left with negative X (left)
                    }

                } else {
                    int closestDirection = 0;
                    double smallestDifference = Double.MAX_VALUE;
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        for (int direction = 0; direction < reefDirections.length; direction++) {
                            // Normalize the difference to be within -180 to +180 degrees
                            double difference = reefDirections[direction] - getState().Pose.getRotation().getDegrees();
                            difference = (difference + 180) % 360 - 180; // Keeps difference in range -180 to +180
                        
                            if (Math.abs(difference) < smallestDifference) {
                                closestDirection = direction;
                                smallestDifference = Math.abs(difference);
                            }
                        }
                    } else {
                        for (int direction = 0; direction < reefDirections.length; direction++) {
                            // Normalize the difference to be within -180 to +180 degrees
                            double difference = reefDirections[direction] - getState().Pose.getRotation().getDegrees() + 180;
                            difference = (difference + 180) % 360 - 180; // Keeps difference in range -180 to +180
                        
                            if (Math.abs(difference) < smallestDifference) {
                                closestDirection = direction;
                                smallestDifference = Math.abs(difference);
                            }
                        }
                    }

                    orientation = reefDirections[closestDirection];
                    if (!(Math.abs(driverController.getRightX()) > 0.2)) {
                        if (Math.abs(getState().Pose.getRotation().getDegrees() - orientation) >= 1) {
                            setControl(new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(7, 0, 0).withTargetDirection(Rotation2d.fromDegrees(orientation)).withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                            .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))); // Drive left with negative X (left)
                        } else {
                            setControl(new SwerveRequest.FieldCentric().withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                            .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))); // Drive left with negative X (left)
                        }
                    } else {
                        setControl(new SwerveRequest.FieldCentric().withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * Math.abs(driverController.getRightX()) * MaxAngularRate));// Drive counterclockwise with negative X (left)
                    }
                }
            }

            @Override
            public void end(boolean interrupted) {
                SmartDashboard.putBoolean("Angle Align", false);
            }
        };
    }

    public boolean isValidTarget(Limelight limelight) {
        try {
            aprilTagID = LimelightHelpers.getFiducialID(limelight.getLimelightName());
            return usedAprilTags.contains((int) aprilTagID);
        } catch (Exception ex) {
            return false;
        }
        
    }


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        SmartDashboard.putNumber("Yaw", getPigeon2().getYaw().getValueAsDouble());
        
        SmartDashboard.putNumber("Drivetrain X", getState().Pose.getTranslation().getX());
        SmartDashboard.putNumber("Drivetrain Y", getState().Pose.getTranslation().getY());
        SmartDashboard.putNumber("Drivetrain Current Direction", getState().Pose.getRotation().getDegrees());

        SmartDashboard.putNumber("L Sensor Measure (MM)", lTOFSensor.getRange());
        SmartDashboard.putNumber("R Sensor Measure (MM)", rTOFSensor.getRange());

        SmartDashboard.putBoolean("L Sensor", getLTOFBeam());
        SmartDashboard.putBoolean("R Sensor", getRTOFBeam());
        
        field.setRobotPose(getState().Pose);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        if (mapleSimSwerveDrivetrain != null)
            DogLog.log("Drive/SimulationPose", MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            // TODO: modify the following constants according to your robot
            Pounds.of(115), // robot weight
            Inches.of(30), // bumper length
            Inches.of(30), // bumper width
            DCMotor.getKrakenX60Foc(1), // drive motor type
            DCMotor.getKrakenX60Foc(1), // steer motor type
            1.2, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);

    }

    // public boolean isLBeamBroken() {
    //     return !lTOFSensor.get();
    // }

    // public boolean isRBeamBroken() {
    //     return !rTOFSensor.get();
    // }

    public void resetPoseSimulationToDrive() {
        if (this.mapleSimSwerveDrivetrain != null)
            MapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(getState().Pose);
        Timer.delay(0.1); // Wait for simulation to update
        super.resetPose(getState().Pose);
    }

    private void configTOFSensors() {
        rTOFSensor.setRangingMode(RangingMode.Short, kNumConfigAttempts);
        lTOFSensor.setRangingMode(RangingMode.Short, kNumConfigAttempts);
        rTOFSensor.setRangeOfInterest( 6, 6, 9, 9);
        lTOFSensor.setRangeOfInterest( 6, 6, 9, 9);
    }

    public boolean getLTOFBeam() {
        return lTOFSensor.getRange() > TOFSensorThreshold; // millimeters
    }

    public boolean getRTOFBeam() {
        return rTOFSensor.getRange() > TOFSensorThreshold; // millimeters
    }

    public void createAlignmentConstants() {
        aprilTagPositions[1][0] = 16.68732537; // X position
        aprilTagPositions[1][1] = 0.6281432563; // Y position
        aprilTagPositions[1][2] = 126; // Angle

        aprilTagPositions[2][0] = 16.68732537; // X position
        aprilTagPositions[2][1] = 7.414274829; // Y position
        aprilTagPositions[2][2] = 234; // Angle

        aprilTagPositions[12][0] = 0.8613157226; // X position
        aprilTagPositions[12][1] = 0.6281432563; // Y position
        aprilTagPositions[12][2] = 54; // Angle

        aprilTagPositions[13][0] = 0.8613157226; // X position
        aprilTagPositions[13][1] = 7.414274829; // Y position
        aprilTagPositions[13][2] = 306; // Angle

        aprilTagPositions[17][0] = 4.073914148; // X position
        aprilTagPositions[17][1] = 3.301244602; // Y position
        aprilTagPositions[17][2] = 60; // Angle

        aprilTagPositions[18][0] = 3.657607315; // X position
        aprilTagPositions[18][1] = 4.020828042; // Y position
        aprilTagPositions[18][2] = 0; // Angle

        aprilTagPositions[19][0] = 4.073914148; // X position
        aprilTagPositions[19][1] = 4.740411481; // Y position
        aprilTagPositions[19][2] = -60; // Angle

        aprilTagPositions[20][0] = 4.904749809; // X position
        aprilTagPositions[20][1] = 4.740411481; // Y position
        aprilTagPositions[20][2] = -120; // Angle

        aprilTagPositions[21][0] = 5.321056642; // X position
        aprilTagPositions[21][1] = 4.020828042; // Y position
        aprilTagPositions[21][2] = -180; // Angle

        aprilTagPositions[22][0] = 4.904749809; // X position
        aprilTagPositions[22][1] = 3.301244602; // Y position
        aprilTagPositions[22][2] = 120; // Angle

        createOffsets(-0.4, 0.4);

        // Coral Reef
        int subtract = 16;
        for (int id = 0; id < aprilTagPoses.length; id++) {
            // Blue IDs

            // Coral Stations
            if (id >= 12 && id <= 13) {
                aprilTagPoses[id][0] = new Pose2d(createPreAdjustments(preCoralStationAdjust, id, 0), createPreAdjustments(preCoralStationAdjust, id, 1), Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id][1] = new Pose2d(aprilTagPositions[id][0], aprilTagPositions[id][1], Rotation2d.fromDegrees(aprilTagPositions[id][2]));
            }
            
            // Reef
            if (id >= 17) {
                aprilTagPoses[id][0] = new Pose2d(createPreAdjustments(preReefAdjust, id, 0), createPreAdjustments(preReefAdjust, id, 1), Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id][1] = new Pose2d(aprilTagPositions[id][0], aprilTagPositions[id][1], Rotation2d.fromDegrees(aprilTagPositions[id][2]));
            }
            
            // Red IDs
            
            // Coral Stations
            if (id >= 1 && id <=2) {
                aprilTagPoses[id][0] = new Pose2d(createPreAdjustments(preCoralStationAdjust, id, 0), createPreAdjustments(preCoralStationAdjust, id, 1), Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id][1] = new Pose2d(aprilTagPositions[id][0], aprilTagPositions[id][1], Rotation2d.fromDegrees(aprilTagPositions[id][2]));
            }
            
            // Reef
            if (id >= 6 && id <= 11) {
                aprilTagPoses[id][0] = new Pose2d(createPreAdjustments(preReefAdjust, id + subtract, 0) + redAdjustX, createPreAdjustments(preReefAdjust, id + subtract, 1) + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id + subtract][2]));
                aprilTagPoses[id][1] = new Pose2d(aprilTagPositions[id + subtract][0] + redAdjustX, aprilTagPositions[id + subtract][1] + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id + subtract][2]));
                subtract = subtract - 2;
            }
            // System.out.println(Rotation2d.fromDegrees(aprilTagPositions[id][2]));
        }
    }
}