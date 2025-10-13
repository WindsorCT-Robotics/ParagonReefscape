// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.lib.Limelight.RectanglePoseArea;
import frc.lib.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.hardware.IPerceptionCamera;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.units.Degrees;
import frc.robot.units.Meters;
import frc.robot.units.Percent;

public class Limelight extends SubsystemBase {
    Alliance alliance;

    private final String limelightName;
    private final IPerceptionCamera camera;
    private final Supplier<Degrees> yawSupplier;
    private Boolean enable = true;
    private Boolean trust = true;
    private Boolean invalidError = false;
    private Boolean fieldError = false;
    private Boolean distanceError = false;
    private PoseEstimate botpose;

    private Field2d visionField = new Field2d();

    private static final RectanglePoseArea field = new RectanglePoseArea(new Translation2d(0.0, 0.0),
            new Translation2d(16.54, 8.02));

    public Limelight(String subsystemName, IPerceptionCamera camera, Supplier<Degrees> yawSupplier) {
        super(subsystemName);

        limelightName = subsystemName;
        this.camera = camera;
        this.yawSupplier = yawSupplier;

        CameraServer.addCamera(camera.getCamera());
    }

    public String printPoseEstimate() {
        StringBuilder sb = new StringBuilder();

        sb.append(botpose.toString());
        sb.append('\n');
        sb.append("Tags Detected: ");
        sb.append(botpose.tagCount);
        sb.append('\n');
        sb.append("Average Tag Area: ");
        sb.append(botpose.avgTagArea);
        sb.append('\n');
        sb.append("Average Tag Distance: ");
        sb.append(botpose.avgTagDist);
        sb.append('\n');
        sb.append("Latency: ");
        sb.append(botpose.latency);

        return sb.toString();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addCloseable(visionField);
        builder.addStringProperty("Pose Estimate Data", this::printPoseEstimate, null);
    }

    @Override
    public void periodic() { // Checking drivetrain data against limelight/apriltag data to confirm accuracy
                             // of robot position, and updates the position accordingly.
                             // 
        botpose = camera.estimateRobotPosition();
        visionField.setRobotPose(botpose.pose);

        if (enable) {
            // Double targetDistanceX =
            // LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getX();
            // Double targetDistanceY =
            // LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getY();
            // Double targetDistanceZ =
            // LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getZ();

            // SmartDashboard.putNumber("Target Distance X", targetDistanceX);
            // SmartDashboard.putNumber("Target Distance Y", targetDistanceY);
            // SmartDashboard.putNumber("Target Distance Z", targetDistanceZ);

            // SmartDashboard.putNumber("tx", tx_output);
            // SmartDashboard.putNumber("ty", ty_output);
            // SmartDashboard.putNumber("ta", ta_output);

            SmartDashboard.putBoolean("Limelight Error", distanceError);
            SmartDashboard.putBoolean("Field Error", fieldError);
            SmartDashboard.putBoolean("Invalid Target", invalidError);

            SmartDashboard.putNumber("Pigeon 2 Gyro", drivetrain.getPigeon2().getYaw().getValueAsDouble());

            LimelightHelpers.SetRobotOrientation(limelightName, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0,
                    0, 0, 0, 0);

            Double tx_output = LimelightHelpers.getTX(limelightName);
            Double ty_output = LimelightHelpers.getTY(limelightName);
            Double ta_output = LimelightHelpers.getTA(limelightName);

            Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getTranslation()
                    .getDistance(new Translation3d()); // Calculates how far away the april tag is
            Double confidence = (targetDistance - 1) / 6;

            if (!(tx_output == 0 && ty_output == 0 && ta_output == 0)) { // If result finds a vaild target then
                                                                         // continues if
                                                                         // statement result.valid
                invalidError = false;
                botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                visionField.setRobotPose(botpose.pose);
                SmartDashboard.putNumber("Number Of Apriltags", botpose.tagCount);
                if (field.isPoseWithinArea(botpose.pose)) {
                    fieldError = false;
                    if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.pose.getTranslation()) < 0.5
                            && Math.abs(
                                    drivetrain.getState().Pose.getRotation().getDegrees()
                                            - botpose.pose.getRotation().getDegrees()) < 3 // Compares
                                                                                           // the
                                                                                           // drivetrain
                                                                                           // aussumed
                                                                                           // position
                                                                                           // to
                                                                                           // the
                                                                                           // limelight's
                                                                                           // assumed
                                                                                           // position
                            || trust
                            || botpose.tagCount > 1) {
                        distanceError = false;
                        trust = false;
                        drivetrain.addVisionMeasurement(
                                botpose.pose,
                                Utils.fpgaToCurrentTime(botpose.timestampSeconds), // Timer.getFPGATimestamp() -
                                                                                   // (result.latency_capture
                                                                                   // / 1000.0) -
                                                                                   // (result.latency_pipeline /
                                                                                   // 1000.0),
                                VecBuilder.fill(confidence, confidence, .01));
                        SmartDashboard.putNumber("Target Distance", targetDistance);
                    } else {
                        distanceError = true;
                    }
                } else {
                    fieldError = true;
                }
            } else {
                invalidError = true;
            }
        }
    }

    public boolean hasDetectedValidTarget() {
        return camera.hasDetectedValidTarget();
    }

    public Optional<Meters> distanceToTarget() {
        return camera.distanceToTarget();
    }

    public Optional<Percent> confidence(Optional<Meters> distanceToTarget) {
        return distanceToTarget.map(distance -> new Percent((distance.asDouble() - 1) / 6));
    }

    public void (boolean trust) {
        this.trust = trust;
    }

    public String getLimelightName() {
        return limelightName;
    }

    @AutoLogOutput(key = "Vision/TagCount")
    public int getTagCount() {
        try {
            // botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
            botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            return botpose.tagCount;
        } catch (Exception e) {
            return 0;
        }
    }

    @AutoLogOutput(key = "Vision/Position3D")
    public Pose2d getCameraPrediction() {
        try {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName).pose;
        } catch (Exception e) {
            return new Pose2d();
        }

    }
}