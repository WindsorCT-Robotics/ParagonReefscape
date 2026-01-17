package frc.robot.apriltag;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class ReefscapeApriltag extends AprilTag {
    public final DriverStation.Alliance alliance;
    public final AprilTagLocation location;

    public ReefscapeApriltag(int ID, Pose3d pose, AprilTagLocation location, DriverStation.Alliance alliance) {
        super(ID, pose);
        this.location = location;
        this.alliance = alliance;
    }
}
