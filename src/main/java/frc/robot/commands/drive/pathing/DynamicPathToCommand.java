package frc.robot.commands.drive.pathing;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DynamicPathToCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<Waypoint> waypoints;
    private final Rotation2d endHeading;
    private final PathConstraints constraints;

    public DynamicPathToCommand(
        CommandSwerveDrivetrain drivetrain, 
        List<Waypoint> waypoints, 
        Rotation2d endHeading, 
        PathConstraints constraints) {
        this.drivetrain = drivetrain;
        this.waypoints = waypoints;
        this.endHeading = endHeading;
        this.constraints = constraints;
    }

    @Override
    public void initialize() {
        /*
        Example of CTRE Following Trajectory: https://www.chiefdelphi.com/t/example-of-wpilib-swervecontrollercommand-following-trajectory-with-ctre-generated-commandswervedrivetrain/502566
        FollowPathCommand Example from Pathplanner Docs: https://pathplanner.dev/pplib-follow-a-single-path.html#manually-create-path-following-commands
        */

        try {
            AutoBuilder.followPath(createPath(waypoints, endHeading, constraints));
        } catch (Exception e) {
            System.out.println("No Command due to try-catch");
        }
    }

    private PathPlannerPath createPath(List<Waypoint> waypoints, Rotation2d endHeading, PathConstraints constraints) {
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, endHeading) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return path;
    }
}