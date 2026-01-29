package frc.robot.apriltag;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
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
            case 1, 2, 12, 13:
                location = AprilTagLocation.SOURCE;
                break;
            case 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22:
                location = AprilTagLocation.REEF;
                break;
            case 4, 5, 14, 15:
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

    public Distance getFieldWidth() {
        return Meters.of(layout.getFieldWidth());
    }

    public Distance getFieldLength() {
        return Meters.of(layout.getFieldLength());
    }
}
