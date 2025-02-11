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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.lib.Limelight.LimelightHelpers;
import frc.lib.Limelight.LimelightHelpers.RawFiducial;

public class ReefAlignCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private PathPlannerPath path;
    private double aprilTagID;
    private List<Waypoint> waypoints;
    private Rotation2d orientation;

    Pose2d[][] blueID = new Pose2d[6][2];
    Pose2d[][] redID = new Pose2d[6][2];

    private final double[] ID_17 = new double[2];
    private final double[] ID_18 = new double[2];
    private final double[] ID_19 = new double[2];
    private final double[] ID_20 = new double[2];
    private final double[] ID_21 = new double[2];
    private final double[] ID_22 = new double[2];

    private final double redAdjustX = 8.58;
    private final double redAdjustY = 0.0;
    private final double preIDAdjust = 0.15;

    public ReefAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        ID_17[0] = 3.84; ID_17[1] = 2.88;
        ID_18[0] = 3.2; ID_18[1] = 4.05;
        ID_19[0] = 3.84; ID_19[1] = 5.16;
        ID_20[0] = 5.15; ID_20[1] = 5.15;
        ID_21[0] = 5.8; ID_21[1] = 4.05;
        ID_22[0] = 5.15; ID_22[1] = 2.9;

        // Blue IDs
        blueID[0][0] = new Pose2d(ID_17[0] - preIDAdjust, ID_17[1] - preIDAdjust, Rotation2d.fromDegrees(60)); // Pre ID 17
        blueID[0][1] = new Pose2d(ID_17[0], ID_17[1], Rotation2d.fromDegrees(60)); // ID 17

        blueID[1][0] = new Pose2d(ID_18[0] - preIDAdjust, ID_18[1], Rotation2d.fromDegrees(0)); // Pre ID 18
        blueID[1][1] = new Pose2d(ID_18[0], ID_18[1], Rotation2d.fromDegrees(0)); // ID 18

        blueID[2][0] = new Pose2d(ID_19[0] - preIDAdjust, ID_19[1] + preIDAdjust, Rotation2d.fromDegrees(-60)); // Pre ID 19
        blueID[2][1] = new Pose2d(ID_19[0], ID_19[1], Rotation2d.fromDegrees(-60)); // ID 19

        blueID[3][0] = new Pose2d(ID_20[0] + preIDAdjust, ID_20[1] + preIDAdjust, Rotation2d.fromDegrees(-120)); // Pre ID 20
        blueID[3][1]= new Pose2d(ID_20[0], ID_20[1], Rotation2d.fromDegrees(-120)); // ID 20

        blueID[4][0] = new Pose2d(ID_21[0] + preIDAdjust, ID_21[1], Rotation2d.fromDegrees(-180)); // Pre ID 21
        blueID[4][1] = new Pose2d(ID_21[0], ID_21[1], Rotation2d.fromDegrees(-180)); // ID 21

        blueID[5][0] = new Pose2d(ID_22[0] + preIDAdjust, ID_22[1] - preIDAdjust, Rotation2d.fromDegrees(120)); // Pre ID 22
        blueID[5][1] = new Pose2d(ID_22[0], ID_22[1], Rotation2d.fromDegrees(120)); // ID 22


        // Red IDs
        redID[0][0] = new Pose2d(ID_17[0] - preIDAdjust + redAdjustX, ID_17[1] - preIDAdjust, Rotation2d.fromDegrees(60)); // Pre ID 11
        redID[0][1] = new Pose2d(ID_17[0] + redAdjustX, ID_17[1], Rotation2d.fromDegrees(60)); // ID 11

        redID[1][0] = new Pose2d(ID_18[0] - preIDAdjust + redAdjustX, ID_18[1], Rotation2d.fromDegrees(0)); // Pre ID 10
        redID[1][1] = new Pose2d(ID_18[0] + redAdjustX, ID_18[1], Rotation2d.fromDegrees(0)); // ID 10

        redID[2][0] = new Pose2d(ID_19[0] - preIDAdjust + redAdjustX, ID_19[1] + preIDAdjust, Rotation2d.fromDegrees(-60)); // Pre ID 9
        redID[2][1] = new Pose2d(ID_19[0] + redAdjustX, ID_19[1], Rotation2d.fromDegrees(-60)); // ID 9

        redID[3][0] = new Pose2d(ID_20[0] + preIDAdjust + redAdjustX, ID_20[1] + preIDAdjust, Rotation2d.fromDegrees(-120)); // Pre ID 8
        redID[3][1]= new Pose2d(ID_20[0] + redAdjustX, ID_20[1], Rotation2d.fromDegrees(-120)); // ID 8

        redID[4][0] = new Pose2d(ID_21[0] + preIDAdjust + redAdjustX, ID_21[1], Rotation2d.fromDegrees(-180)); // Pre ID 7
        redID[4][1] = new Pose2d(ID_21[0] + redAdjustX, ID_21[1], Rotation2d.fromDegrees(-180)); // ID 7

        redID[5][0] = new Pose2d(ID_22[0] + preIDAdjust + redAdjustX, ID_22[1] - preIDAdjust, Rotation2d.fromDegrees(120)); // Pre ID 6
        redID[5][1] = new Pose2d(ID_22[0] + redAdjustX, ID_22[1], Rotation2d.fromDegrees(120)); // ID 6
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
        aprilTagID = 22.0;
        if (aprilTagID != 0.0) {
            // Create a list of waypoints from poses. Each pose represents one waypoint.
            // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.

            trajectory();
            PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
            // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

            // Create the path using the waypoints created above
            path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                    new GoalEndState(0.0, blueID[5][1].getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );

            // Prevent the path from being flipped if the coordinates are already correct
            path.preventFlipping = true;

            drivetrain.followPathCommand(path).schedule();   
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
}

