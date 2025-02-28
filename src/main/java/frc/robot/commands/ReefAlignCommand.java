package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.lib.Limelight.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import java.util.List;
import java.util.Set;

public class ReefAlignCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final CommandXboxController op;
    private final CommandXboxController drive;
    private final boolean isCoralStation;
    private final String direction;
    private PathPlannerPath path;
    private double aprilTagID;
    private List<Waypoint> waypoints;
    private Rotation2d orientation;
    
    Pose2d[][] aprilTagPoses = new Pose2d[23][2];
    double[][] aprilTagPositions = new double[23][3];
    private static final Set<Integer> usedAprilTags = Set.of(1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22);

    private final double redAdjustX = 8.569452;
    private final double redAdjustY = 0.0;
    private final double preReefAdjust = -0.3;
    private final double preCoralStationAdjust = 0.3;

    private final double leftAngle = 90.0;
    private final double rightAngle = -90.0;

    private final double branchOffset = 0.1651;

    public ReefAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController op, CommandXboxController drive, boolean isCoralStation, String direction) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.op = op;
        this.drive = drive;
        this.direction = direction;
        this.isCoralStation = isCoralStation;
        
        // Tuned measurements
        // ID_17[0] = 3.94; ID_17[1] = 3.08; 
        // ID_18[0] = 3.2 + 0.172 - 0.0508; ID_18[1] = 4.05;
        // ID_19[0] = 3.84 + 0.1016; ID_19[1] = 5.16 - 0.1016;
        // ID_20[0] = 5.15 - 0.1016; ID_20[1] = 5.15 - 0.1016;
        // ID_21[0] = 5.8 - 0.1778; ID_21[1] = 4.05;
        // ID_22[0] = 5.15 - 0.127; ID_22[1] = 2.9 + 0.127;
        
        // Measurements of actual april tags

        aprilTagPositions[1][0] = 16.697198;
        aprilTagPositions[1][1] = 0.65532;
        aprilTagPositions[1][2] = 126;

        aprilTagPositions[2][0] = 16.697198;
        aprilTagPositions[2][1] = 7.39648;
        aprilTagPositions[2][2] = 234;

        aprilTagPositions[12][0] = 0.851154; // X position
        aprilTagPositions[12][1] = 0.65532; // Y position
        aprilTagPositions[12][2] = 54; // Angle

        aprilTagPositions[13][0] = 0.851154; // X position
        aprilTagPositions[13][1] = 7.39648; // Y position
        aprilTagPositions[13][2] = 306; // Angle

        aprilTagPositions[17][0] = 4.073906; // X position
        aprilTagPositions[17][1] = 3.306318; // Y position
        aprilTagPositions[17][2] = 60; // Angle

        aprilTagPositions[18][0] = 3.6576; // X position
        aprilTagPositions[18][1] = 4.0259; // Y position
        aprilTagPositions[18][2] = 0; // Angle

        aprilTagPositions[19][0] = 4.073906; // X position
        aprilTagPositions[19][1] = 4.745482; // Y position
        aprilTagPositions[19][2] = -60; // Angle

        aprilTagPositions[20][0] = 4.90474; // X position
        aprilTagPositions[20][1] = 4.745482; // Y position
        aprilTagPositions[20][2] = -120; // Angle

        aprilTagPositions[21][0] = 5.321046; // X position
        aprilTagPositions[21][1] = 4.0259; // Y position
        aprilTagPositions[21][2] = -180; // Angle

        aprilTagPositions[22][0] = 4.90474; // X position
        aprilTagPositions[22][1] = 3.3063434; // Y position
        aprilTagPositions[22][2] = 120; // Angle
        
        createOffsets(-0.4, 0.4); // Meters
        
        // Coral Reef
        int subtract = 6;
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
            if (id - subtract >= 6 && id - subtract <= 11) {
                aprilTagPoses[id - subtract][0] = new Pose2d(createPreAdjustments(preReefAdjust, id, 0) + redAdjustX, createPreAdjustments(preReefAdjust, id, 1) + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id - subtract][1] = new Pose2d(aprilTagPositions[id][0] + redAdjustX, aprilTagPositions[id][1] + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                subtract = subtract + 2;
            }
            System.out.println(Rotation2d.fromDegrees(aprilTagPositions[id][2]));
        }
    }

    private void trajectory() {
        // Checks if the id that is being used is an id that is allowed to be used for positioning
        if (!usedAprilTags.contains((int) aprilTagID)) {
            return;
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
                if (drivetrain.getState().Pose.getY() < 4.025) {
                    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                        aprilTagID = 12.0;
                    } else {
                        aprilTagID = 1.0;
                    }
                }
                
                if (drivetrain.getState().Pose.getY() > 4.025) {
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
    }

    @Override
    public void initialize() {
        aprilTagID = LimelightHelpers.getFiducialID(limelight.getLimelightName());
        if (aprilTagID > 0.0) {
            // Create a list of waypoints from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
            orientation = Rotation2d.fromDegrees(aprilTagPoses[(int) aprilTagID][0].getRotation().getDegrees());

            trajectory();

            if (waypoints == null) {
                System.out.println("Waypoints is null");
                return;
            }

            PathConstraints constraints = new PathConstraints(0.5, 0.5, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

            // Create the path using the waypoints created above
            path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, orientation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;
            System.out.println(waypoints);
            drivetrain.followPathCommand(path).until(drive.x()).schedule();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
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
}