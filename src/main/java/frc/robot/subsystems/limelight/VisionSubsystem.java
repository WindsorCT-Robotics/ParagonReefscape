package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import frc.lib.Limelight.RectanglePoseArea;
import frc.robot.hardware.IPerceptionCamera;
import frc.robot.result.Failure;
import frc.robot.result.Result;
import frc.robot.result.Success;

public class VisionSubsystem extends SubsystemBase {
    public sealed interface VisionError permits NoTags, PoseOutOfBounds { }
    public record NoTags() implements VisionError { }
    public record PoseOutOfBounds(PoseEstimate invalidPose) implements VisionError { }
    
    private final IPerceptionCamera camera;
    private static final Distance FIELD_LENGTH = Meter.of(16.54);
    private static final Distance FIELD_WIDTH = Meter.of(8.02);
    private static final RectanglePoseArea field = new RectanglePoseArea(
        new Translation2d(Meter.of(0), Meter.of(0))
        , new Translation2d(
            FIELD_LENGTH
            , FIELD_WIDTH));
    
    public VisionSubsystem(String subsystemName, IPerceptionCamera camera) {
        super(subsystemName);
        
        this.camera = camera;
        CameraServer.addCamera(camera.getCamera());
    }

    public Result<Pose2d, VisionError> estimatePosition(Angle yaw) {
        PoseEstimate result;

        result = camera.estimateRobotPosition(yaw);

        if (result.tagCount < 1) {
            return new Failure<>(new NoTags());
        }

        if (!field.isPoseWithinArea(result.pose)) {
            return new Failure<>(new PoseOutOfBounds(result));
        }

        return new Success<>(result.pose);
    }
}
