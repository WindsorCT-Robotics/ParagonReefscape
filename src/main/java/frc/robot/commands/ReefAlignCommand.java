package frc.robot.commands;


import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.lib.Limelight.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import java.util.List;

public class ReefAlignCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final CommandXboxController op;
    private final String direction;
    private PathPlannerPath path;
    private double aprilTagID;
    private List<Waypoint> waypoints;
    private Rotation2d orientation;

    Pose2d[][] aprilTagPoses = new Pose2d[23][2];
    double[][] aprilTagPositions = new double[23][3];

    private final double redAdjustX = 8.569452;
    private final double redAdjustY = 0.0;
    private final double preIDAdjust = -0.3;

    private final double leftAngle = 90.0;
    private final double rightAngle = -90.0;

    private final double branchOffset = 0.1651;

    public ReefAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController op, String direction) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.op = op;
        this.direction = direction;
        
        // Tuned measurements
        // ID_17[0] = 3.94; ID_17[1] = 3.08; 
        // ID_18[0] = 3.2 + 0.172 - 0.0508; ID_18[1] = 4.05;
        // ID_19[0] = 3.84 + 0.1016; ID_19[1] = 5.16 - 0.1016;
        // ID_20[0] = 5.15 - 0.1016; ID_20[1] = 5.15 - 0.1016;
        // ID_21[0] = 5.8 - 0.1778; ID_21[1] = 4.05;
        // ID_22[0] = 5.15 - 0.127; ID_22[1] = 2.9 + 0.127;

        // Measurements of actual april tags

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

        createOffsets(-0.4); // Meters

        int subtract = 6;
        for (int id = 17; id < 23; id++) {
            // Blue IDs
            if (id >= 17) {
                aprilTagPoses[id][0] = new Pose2d(createPreAdjustments(preIDAdjust, id, 0), createPreAdjustments(preIDAdjust, id, 1), Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id][1] = new Pose2d(aprilTagPositions[id][0], aprilTagPositions[id][1], Rotation2d.fromDegrees(aprilTagPositions[id][2]));
            }
            // Red IDs
            if (id - subtract >= 6 && id - subtract <= 11) {
                aprilTagPoses[id - subtract][0] = new Pose2d(createPreAdjustments(preIDAdjust, id, 0) + redAdjustX, createPreAdjustments(preIDAdjust, id, 1) + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                aprilTagPoses[id - subtract][1] = new Pose2d(aprilTagPositions[id][0] + redAdjustX, aprilTagPositions[id][1] + redAdjustY, Rotation2d.fromDegrees(aprilTagPositions[id][2]));
                subtract = subtract + 2;
            }

            System.out.println(Rotation2d.fromDegrees(aprilTagPositions[id][2]));
        }
            
        }

        // // Blue IDs
        // aprilTagPoses[17][0] = new Pose2d(createPreAdjustments(preIDAdjust, 0, 0), createPreAdjustments(preIDAdjust, 0, 1), Rotation2d.fromDegrees(aprilTagPositions[0][2])); // Pre ID 17
        // aprilTagPoses[17][1] = new Pose2d(aprilTagPositions[17][0], aprilTagPositions[17][1], Rotation2d.fromDegrees(aprilTagPositions[17][2])); // ID 17

        // aprilTagPoses[18][0] = new Pose2d(createPreAdjustments(preIDAdjust, 1, 0), aprilTagPositions[1][1], Rotation2d.fromDegrees(aprilTagPositions[1][2])); // Pre ID 18
        // aprilTagPoses[18][1] = new Pose2d(aprilTagPositions[1][0], aprilTagPositions[1][1], Rotation2d.fromDegrees(aprilTagPositions[1][2])); // ID 18

        // aprilTagPoses[19][0] = new Pose2d(createPreAdjustments(preIDAdjust, 2, 0), createPreAdjustments(preIDAdjust, 2, 1), Rotation2d.fromDegrees(aprilTagPositions[2][2])); // Pre ID 19
        // aprilTagPoses[19][1] = new Pose2d(aprilTagPositions[2][0], aprilTagPositions[2][1], Rotation2d.fromDegrees(aprilTagPositions[2][2])); // ID 19

        // aprilTagPoses[20][0] = new Pose2d(createPreAdjustments(preIDAdjust, 3, 0), createPreAdjustments(preIDAdjust, 3, 1), Rotation2d.fromDegrees(aprilTagPositions[3][2])); // Pre ID 20
        // aprilTagPoses[20][1] = new Pose2d(aprilTagPositions[3][0], aprilTagPositions[3][1], Rotation2d.fromDegrees(aprilTagPositions[3][2])); // ID 20

        // aprilTagPoses[21][0] = new Pose2d(createPreAdjustments(preIDAdjust, 4, 0), aprilTagPositions[4][1], Rotation2d.fromDegrees(aprilTagPositions[4][2])); // Pre ID 21
        // aprilTagPoses[21][1] = new Pose2d(aprilTagPositions[4][0], aprilTagPositions[4][1], Rotation2d.fromDegrees(aprilTagPositions[4][2])); // ID 21

        // aprilTagPoses[22][0] = new Pose2d(createPreAdjustments(preIDAdjust, 5, 0), createPreAdjustments(preIDAdjust, 5, 1), Rotation2d.fromDegrees(aprilTagPositions[5][2])); // Pre ID 22
        // aprilTagPoses[22][1] = new Pose2d(aprilTagPositions[5][0], aprilTagPositions[5][1], Rotation2d.fromDegrees(aprilTagPositions[5][2])); // ID 22


        // // Red IDs
        // aprilTagPoses[11][0] = new Pose2d(aprilTagPositions[0][0] - createPreAdjustments(preIDAdjust, 0, 0) + redAdjustX, aprilTagPositions[0][1] - createPreAdjustments(preIDAdjust, 0, 1), Rotation2d.fromDegrees(aprilTagPositions[0][2])); // Pre ID 11
        // aprilTagPoses[11][1] = new Pose2d(aprilTagPositions[0][0] + redAdjustX, aprilTagPositions[0][1], Rotation2d.fromDegrees(aprilTagPositions[0][2])); // ID 11

        // aprilTagPoses[10][0] = new Pose2d(aprilTagPositions[1][0] - createPreAdjustments(preIDAdjust, 1, 0) + redAdjustX, aprilTagPositions[1][1], Rotation2d.fromDegrees(aprilTagPositions[1][2])); // Pre ID 10
        // aprilTagPoses[10][1] = new Pose2d(aprilTagPositions[1][0] + redAdjustX, aprilTagPositions[1][1], Rotation2d.fromDegrees(aprilTagPositions[1][2])); // ID 10

        // aprilTagPoses[9][0] = new Pose2d(aprilTagPositions[2][0] - createPreAdjustments(preIDAdjust, 2, 0) + redAdjustX, aprilTagPositions[2][1] + createPreAdjustments(preIDAdjust, 2, 1), Rotation2d.fromDegrees(aprilTagPositions[2][2])); // Pre ID 9
        // aprilTagPoses[9][1] = new Pose2d(aprilTagPositions[2][0] + redAdjustX, aprilTagPositions[2][1], Rotation2d.fromDegrees(aprilTagPositions[2][2])); // ID 9

        // aprilTagPoses[8][0] = new Pose2d(aprilTagPositions[3][0] + createPreAdjustments(preIDAdjust, 3, 0) + redAdjustX, aprilTagPositions[3][1] + createPreAdjustments(preIDAdjust, 3, 1), Rotation2d.fromDegrees(aprilTagPositions[3][2])); // Pre ID 8
        // aprilTagPoses[8][1] = new Pose2d(aprilTagPositions[3][0] + redAdjustX, aprilTagPositions[3][1], Rotation2d.fromDegrees(aprilTagPositions[3][2])); // ID 8

        // aprilTagPoses[7][0] = new Pose2d(aprilTagPositions[4][0] + createPreAdjustments(preIDAdjust, 4, 0) + redAdjustX, aprilTagPositions[4][1], Rotation2d.fromDegrees(aprilTagPositions[4][2])); // Pre ID 7
        // aprilTagPoses[7][1] = new Pose2d(aprilTagPositions[4][0] + redAdjustX, aprilTagPositions[4][1], Rotation2d.fromDegrees(aprilTagPositions[4][2])); // ID 7

        // aprilTagPoses[6][0] = new Pose2d(aprilTagPositions[5][0] + createPreAdjustments(preIDAdjust, 5, 0) + redAdjustX, aprilTagPositions[5][1] - createPreAdjustments(preIDAdjust, 5, 1), Rotation2d.fromDegrees(aprilTagPositions[5][2])); // Pre ID 6
        // aprilTagPoses[6][1] = new Pose2d(aprilTagPositions[5][0] + redAdjustX, aprilTagPositions[5][1], Rotation2d.fromDegrees(aprilTagPositions[5][2])); // ID 6

    private void trajectory() {
        double[] prePose = {aprilTagPoses[(int) aprilTagID][0].getX(), aprilTagPoses[(int) aprilTagID][0].getY()};
        double[] pose = {aprilTagPoses[(int) aprilTagID][1].getX(), aprilTagPoses[(int) aprilTagID][1].getY()};

        if (direction.equalsIgnoreCase("left")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(calculateDirectionalTranslation(prePose[0], branchOffset, orientation.getDegrees() + leftAngle, "x"), calculateDirectionalTranslation(prePose[1], branchOffset, orientation.getDegrees() + leftAngle, "y"), orientation), 
                new Pose2d(calculateDirectionalTranslation(pose[0], branchOffset, orientation.getDegrees() + leftAngle, "x"), calculateDirectionalTranslation(pose[1], branchOffset, orientation.getDegrees() + leftAngle, "y"), orientation));
        }

        if (direction.equalsIgnoreCase("center")) {
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
            drivetrain.followPathCommand(path).until(op.x()).schedule();
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

    public void createOffsets(double distance) {
        for (int row = 0; row < aprilTagPositions.length; row++) {
            for (int col = 0; col < aprilTagPositions[row].length - 1; col++) {
                if (col == 0) {
                    aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], distance, aprilTagPositions[row][2], "x"); // X coordinate
                } else {
                    aprilTagPositions[row][col] = calculateDirectionalTranslation(aprilTagPositions[row][col], distance, aprilTagPositions[row][2], "y"); // Y coordinate
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

