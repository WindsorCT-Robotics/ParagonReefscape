package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
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
    private final Supplier<Angle> yawSupplier;
    private static final Distance FIELD_LENGTH = Meters.of(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getFieldLength());
    private static final Distance FIELD_WIDTH = Meters.of(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getFieldWidth());
    private static final RectanglePoseArea field = new RectanglePoseArea(
        new Translation2d(Meter.of(0), Meter.of(0))
        , new Translation2d(
            FIELD_LENGTH
            , FIELD_WIDTH));
    
    public VisionSubsystem(String subsystemName, IPerceptionCamera camera, Supplier<Angle> yawSupplier) {
        super(subsystemName);
        
        this.camera = camera;
        CameraServer.addCamera(camera.getCamera());
        this.yawSupplier = yawSupplier;
    }

    private Result<PoseEstimate, VisionError> estimatePosition() {
        PoseEstimate result;

        result = camera.estimateRobotPosition(yawSupplier.get());

        if (result.tagCount < 1) {
            return new Failure<>(new NoTags());
        }

        if (!field.isPoseWithinArea(result.pose)) {
            return new Failure<>(new PoseOutOfBounds(result));
        }

        return new Success<>(result);
    }

    public Supplier<Result<PoseEstimate, VisionError>> positionEstimateSupplier() {
        return this::estimatePosition;
    }
}
