package frc.robot.hardware;

import java.util.Optional;

import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;

/**
 * Provides a common interface for a camera capable of perceiving its
 * environment, including estimating distance and position.
 */
public interface IPerceptionCamera extends ICamera {
    PoseEstimate estimateRobotPosition(Angle yaw);

    boolean isEnabled();

    boolean hasDetectedValidTarget();

    Optional<Distance> distanceToTarget();
}
