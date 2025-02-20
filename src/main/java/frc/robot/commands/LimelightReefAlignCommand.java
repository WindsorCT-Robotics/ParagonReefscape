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

public class LimelightReefAlignCommand extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final CommandXboxController op;
    private final String direction;
    private PathPlannerPath path;
    private double aprilTagID;
    private List<Waypoint> waypoints;
    private Rotation2d orientation;
    private String ll;

    private final double redAdjustX = 8.58;
    private final double redAdjustY = 0.0;
    private final double preIDAdjust = -0.5;
    private final double offset = 0.0;

    private final double leftAngle = -90.0;
    private final double rightAngle = 90.0;

    public LimelightReefAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight, CommandXboxController op, String direction) {
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

        ll = limelight.getLimelightName();
    }

    private double[] getTargetIDCoordinates() {
        double x = LimelightHelpers.getTargetPose3d_RobotSpace(ll).getX();
        double y = LimelightHelpers.getTargetPose3d_RobotSpace(ll).getY();
        double[] coordinates = {x, y};
        return coordinates;
    }

    private double getTargetIDAngle() {
        double angle = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getRotation().getAngle();
        return angle;
    }

    private void trajectory() {
        double[] coordinates = getTargetIDCoordinates();
        double angle = getTargetIDAngle();
        Rotation2d orientation = Rotation2d.fromDegrees(getTargetIDAngle());

        if (direction.equalsIgnoreCase("left")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset + preIDAdjust, angle + leftAngle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset + preIDAdjust, angle, "y"), orientation),  // Pre Pose
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset, angle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset, angle, "y"), orientation)); // Pose
        }

        if (direction.equalsIgnoreCase("right")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset + preIDAdjust, angle + rightAngle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset + preIDAdjust, angle, "y"), orientation),  // Pre Pose
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset, angle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset, angle, "y"), orientation)); // Pose
        }

        if (direction.equalsIgnoreCase("center")) {
            waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset + preIDAdjust, angle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset + preIDAdjust, angle, "y"), orientation),  // Pre Pose
            new Pose2d(calculateDirectionalTranslation(coordinates[0], offset, angle, "x"), 
            calculateDirectionalTranslation(coordinates[1], offset, angle, "y"), orientation)); // Pose
        }
        }

    @Override
    public void initialize() {
        aprilTagID = LimelightHelpers.getFiducialID(ll);
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
}

