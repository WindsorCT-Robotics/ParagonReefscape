package frc.robot.commands.drive.pathing;

import java.util.List;

import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathTo extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<Waypoint> waypoints;
    private final Rotation2d endHeading;

    public PathTo(CommandSwerveDrivetrain drivetrain, List<Waypoint> waypoints, Rotation2d endHeading) {
        this.drivetrain = drivetrain;
        this.waypoints = waypoints;
        this.endHeading = endHeading;
    }

    @Override
    public void initialize() {
        
    }
}
