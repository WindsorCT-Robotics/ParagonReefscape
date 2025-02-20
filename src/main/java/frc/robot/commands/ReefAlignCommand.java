package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.lib.Limelight.LimelightHelpers;
import frc.lib.Limelight.LimelightHelpers.RawFiducial;

public class ReefAlignCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final CommandXboxController op;
    private PathPlannerPath path;
    private double aprilTagID;
    private List<Waypoint> waypoints;
    private Rotation2d orientation;

    Pose2d[][] blueID = new Pose2d[6][2];
    Pose2d[][] redID = new Pose2d[6][2];

    // Measurements of actual april tags

    private double[][] aprilTagPoses = {
        {4.073906, 3.306318, 60}, // ID_17
        {3.6576, 4.0259, 0},     // ID_18
        {4.073906, 4.745482, -60}, // ID_19
        {4.90474, 4.745482, -120},  // ID_20
        {5.321046, 4.0259, -180},   // ID_21
        {4.90474, 3.3063434, 120}  // ID_22
    };

    private final double redAdjustX = 8.569452;
    private final double redAdjustY = 0.0;
    private final double preIDAdjust = -0.5;

    public ReefAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController op) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.op = op;
        
        // Tuned measurements
        // ID_17[0] = 3.94; ID_17[1] = 3.08; 
        // ID_18[0] = 3.2 + 0.172 - 0.0508; ID_18[1] = 4.05;
        // ID_19[0] = 3.84 + 0.1016; ID_19[1] = 5.16 - 0.1016;
        // ID_20[0] = 5.15 - 0.1016; ID_20[1] = 5.15 - 0.1016;
        // ID_21[0] = 5.8 - 0.1778; ID_21[1] = 4.05;
        // ID_22[0] = 5.15 - 0.127; ID_22[1] = 2.9 + 0.127;

        createOffsets(-0.4); // Meters

        // Blue IDs
        blueID[0][0] = new Pose2d(createPreAdjustments(preIDAdjust, 0, 0), createPreAdjustments(preIDAdjust, 0, 1), Rotation2d.fromDegrees(aprilTagPoses[0][2])); // Pre ID 17
        blueID[0][1] = new Pose2d(aprilTagPoses[0][0], aprilTagPoses[0][1], Rotation2d.fromDegrees(aprilTagPoses[0][2])); // ID 17

        blueID[1][0] = new Pose2d(createPreAdjustments(preIDAdjust, 1, 0), aprilTagPoses[1][1], Rotation2d.fromDegrees(aprilTagPoses[1][2])); // Pre ID 18
        blueID[1][1] = new Pose2d(aprilTagPoses[1][0], aprilTagPoses[1][1], Rotation2d.fromDegrees(aprilTagPoses[1][2])); // ID 18

        blueID[2][0] = new Pose2d(createPreAdjustments(preIDAdjust, 2, 0), createPreAdjustments(preIDAdjust, 2, 1), Rotation2d.fromDegrees(aprilTagPoses[2][2])); // Pre ID 19
        blueID[2][1] = new Pose2d(aprilTagPoses[2][0], aprilTagPoses[2][1], Rotation2d.fromDegrees(aprilTagPoses[2][2])); // ID 19

        blueID[3][0] = new Pose2d(createPreAdjustments(preIDAdjust, 3, 0), createPreAdjustments(preIDAdjust, 3, 1), Rotation2d.fromDegrees(aprilTagPoses[3][2])); // Pre ID 20
        blueID[3][1] = new Pose2d(aprilTagPoses[3][0], aprilTagPoses[3][1], Rotation2d.fromDegrees(aprilTagPoses[3][2])); // ID 20

        blueID[4][0] = new Pose2d(createPreAdjustments(preIDAdjust, 4, 0), aprilTagPoses[4][1], Rotation2d.fromDegrees(aprilTagPoses[4][2])); // Pre ID 21
        blueID[4][1] = new Pose2d(aprilTagPoses[4][0], aprilTagPoses[4][1], Rotation2d.fromDegrees(aprilTagPoses[4][2])); // ID 21

        blueID[5][0] = new Pose2d(createPreAdjustments(preIDAdjust, 5, 0), createPreAdjustments(preIDAdjust, 5, 1), Rotation2d.fromDegrees(aprilTagPoses[5][2])); // Pre ID 22
        blueID[5][1] = new Pose2d(aprilTagPoses[5][0], aprilTagPoses[5][1], Rotation2d.fromDegrees(aprilTagPoses[5][2])); // ID 22


        // Red IDs
        redID[0][0] = new Pose2d(aprilTagPoses[0][0] - createPreAdjustments(preIDAdjust, 0, 0) + redAdjustX, aprilTagPoses[0][1] - createPreAdjustments(preIDAdjust, 0, 1), Rotation2d.fromDegrees(aprilTagPoses[0][2])); // Pre ID 11
        redID[0][1] = new Pose2d(aprilTagPoses[0][0] + redAdjustX, aprilTagPoses[0][1], Rotation2d.fromDegrees(aprilTagPoses[0][2])); // ID 11

        redID[1][0] = new Pose2d(aprilTagPoses[1][0] - createPreAdjustments(preIDAdjust, 1, 0) + redAdjustX, aprilTagPoses[1][1], Rotation2d.fromDegrees(aprilTagPoses[1][2])); // Pre ID 10
        redID[1][1] = new Pose2d(aprilTagPoses[1][0] + redAdjustX, aprilTagPoses[1][1], Rotation2d.fromDegrees(aprilTagPoses[1][2])); // ID 10

        redID[2][0] = new Pose2d(aprilTagPoses[2][0] - createPreAdjustments(preIDAdjust, 2, 0) + redAdjustX, aprilTagPoses[2][1] + createPreAdjustments(preIDAdjust, 2, 1), Rotation2d.fromDegrees(aprilTagPoses[2][2])); // Pre ID 9
        redID[2][1] = new Pose2d(aprilTagPoses[2][0] + redAdjustX, aprilTagPoses[2][1], Rotation2d.fromDegrees(aprilTagPoses[2][2])); // ID 9

        redID[3][0] = new Pose2d(aprilTagPoses[3][0] + createPreAdjustments(preIDAdjust, 3, 0) + redAdjustX, aprilTagPoses[3][1] + createPreAdjustments(preIDAdjust, 3, 1), Rotation2d.fromDegrees(aprilTagPoses[3][2])); // Pre ID 8
        redID[3][1]= new Pose2d(aprilTagPoses[3][0] + redAdjustX, aprilTagPoses[3][1], Rotation2d.fromDegrees(aprilTagPoses[3][2])); // ID 8

        redID[4][0] = new Pose2d(aprilTagPoses[4][0] + createPreAdjustments(preIDAdjust, 4, 0) + redAdjustX, aprilTagPoses[4][1], Rotation2d.fromDegrees(aprilTagPoses[4][2])); // Pre ID 7
        redID[4][1] = new Pose2d(aprilTagPoses[4][0] + redAdjustX, aprilTagPoses[4][1], Rotation2d.fromDegrees(aprilTagPoses[4][2])); // ID 7

        redID[5][0] = new Pose2d(aprilTagPoses[5][0] + createPreAdjustments(preIDAdjust, 5, 0) + redAdjustX, aprilTagPoses[5][1] - createPreAdjustments(preIDAdjust, 5, 1), Rotation2d.fromDegrees(aprilTagPoses[5][2])); // Pre ID 6
        redID[5][1] = new Pose2d(aprilTagPoses[5][0] + redAdjustX, aprilTagPoses[5][1], Rotation2d.fromDegrees(aprilTagPoses[5][2])); // ID 6
    }

    private void trajectory() {
        int iAprilTagID = (int) aprilTagID;
        switch (iAprilTagID) {
        case 6:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[5][0], redID[5][1]);
            orientation = redID[5][1].getRotation();
            break;

        case 7:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[4][0], redID[4][1]);
            orientation = redID[4][1].getRotation();
            break;

        case 8:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[3][0], redID[3][1]);
            orientation = redID[3][1].getRotation();
            break;

        case 9:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[2][0], redID[2][1]);
            orientation = redID[2][1].getRotation();
            break;

        case 10:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[1][0], redID[1][1]);
            orientation = redID[1][1].getRotation();
            break;

        case 11:
            waypoints = PathPlannerPath.waypointsFromPoses(redID[0][0], redID[0][1]);
            orientation = redID[0][1].getRotation();
            break;

        case 17:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[0][0], blueID[0][1]);
            orientation = blueID[0][1].getRotation();
            break;

        case 18:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[1][0], blueID[1][1]);
            orientation = blueID[1][1].getRotation();
            break;

        case 19:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[2][0], blueID[2][1]);
            orientation = blueID[2][1].getRotation();
            break;

        case 20:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[3][0], blueID[3][1]);
            orientation = blueID[3][1].getRotation();
            break;

        case 21:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[4][0], blueID[4][1]);
            orientation = blueID[4][1].getRotation();
            break;
            
        case 22:
            waypoints = PathPlannerPath.waypointsFromPoses(blueID[5][0], blueID[5][1]);
            orientation = blueID[5][1].getRotation();
            break;
        // case -1:
        //     waypoints = PathPlannerPath.waypointsFromPoses(new Pose2d(drivetrain.getState().Pose.getTranslation().getX(), drivetrain.getState().Pose.getTranslation().getY(), Rotation2d.fromDegrees(drivetrain.getState().Pose.getRotation().getDegrees())));
        }
    }

    @Override
    public void initialize() {
        aprilTagID = LimelightHelpers.getFiducialID(limelight.getLimelightName());
        if (aprilTagID != 0.0) {
            // Create a list of waypoints from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.

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
        for (int row = 0; row < aprilTagPoses.length; row++) {
            for (int col = 0; col < aprilTagPoses[row].length - 1; col++) {
                if (col == 0) {
                    aprilTagPoses[row][col] = calculateDirectionalTranslation(aprilTagPoses[row][col], distance, aprilTagPoses[row][2], "x"); // X coordinate
                } else {
                    aprilTagPoses[row][col] = calculateDirectionalTranslation(aprilTagPoses[row][col], distance, aprilTagPoses[row][2], "y"); // Y coordinate
                }
            }
        }
    }

    public double createPreAdjustments(double distance, int row, int col) {
        if (col == 0) {
            return calculateDirectionalTranslation(aprilTagPoses[row][col], distance, aprilTagPoses[row][2], "x"); // X coordinate
        } else {
            return calculateDirectionalTranslation(aprilTagPoses[row][col], distance, aprilTagPoses[row][2], "y"); // Y coordinate
        }
    }
}

