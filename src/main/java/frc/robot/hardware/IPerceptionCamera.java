package frc.robot.hardware;

import java.util.Optional;

import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.units.Degrees;
import frc.robot.units.Meters;

public interface IPerceptionCamera extends ICamera {

    PoseEstimate estimateRobotPosition(Degrees yaw);

    boolean isEnabled();

    boolean hasDetectedValidTarget();

    Optional<Meters> distanceToTarget();
}
