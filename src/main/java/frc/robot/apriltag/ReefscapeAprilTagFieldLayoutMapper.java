package frc.robot.apriltag;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;

public final class ReefscapeAprilTagFieldLayoutMapper {
    private final AprilTagFieldLayout layout;
    
    public ReefscapeAprilTagFieldLayoutMapper() {
        AprilTagFieldLayout original = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        double fieldLength = original.getFieldLength();
        double fieldWidth = original.getFieldWidth();
        layout = new AprilTagFieldLayout(
                original.getTags().stream().map(ReefscapeAprilTagFieldLayoutMapper::mapTag).toList(),
                fieldLength,
                fieldWidth
        );

        layout.setOrigin(original.getOrigin());
    }

    private static AprilTag mapTag (AprilTag tag) {
        DriverStation.Alliance alliance = tag.ID < 12 ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue;
        AprilTagLocation location;

        switch (tag.ID) {
            case 1:
            case 2:
            case 12:
            case 13:
                location = AprilTagLocation.SOURCE;
                break;
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
            case 22:
                location = AprilTagLocation.REEF;
                break;
            case 4:
            case 5:
            case 14:
            case 15:
                location = AprilTagLocation.BARGE;
                break;
            default:
                location = AprilTagLocation.PROCESSOR;
                break;
        }

        return new ReefscapeApriltag(tag, location, alliance);
    }

    public List<ReefscapeApriltag> getTags() {
        return layout.getTags().stream()
                .map(ReefscapeApriltag.class::cast)
                .toList();
    }
}
