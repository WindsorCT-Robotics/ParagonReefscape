// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Util.RectanglePoseArea;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;
  private String ll = "limelight";
  private Boolean enable = false;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private int validError = 0;
  private int enableError = 0;
  private Pose2d botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(17.55, 8.05));

  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    SmartDashboard.putNumber("Field Error", fieldError);
    SmartDashboard.putNumber("Limelight Error", distanceError);
    SmartDashboard.putNumber("Validity Error", validError);
    SmartDashboard.putNumber("Enable Error", enableError);
    LimelightHelpers.setCameraPose_RobotSpace("", 
      -0.35,    // Forward offset (meters)
      0.0,    // Side offset (meters)
      0.1524,    // Height offset (meters)
      0.0,    // Roll (degrees)
      180.0,   // Pitch (degrees)
      0.0     // Yaw (degrees)
    );
    LimelightHelpers.SetRobotOrientation(ll, drivetrain.getRotation3d().getZ() * 180 / Math.PI, 0, 0, 0, 0, 0);
  }

  @Override
  public void periodic() {
    LimelightHelpers.SetRobotOrientation(ll, drivetrain.getRotation3d().getZ() * 180 / Math.PI, 0, 0, 0, 0, 0);
    if (enable) {
      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d());
      Double confidence = 1 - ((targetDistance - 1) / 6);
      LimelightHelpers.LimelightResults result = LimelightHelpers.getLatestResults(ll);
      if (LimelightHelpers.getRawFiducials(ll).length > 0) {
        botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll).pose;
        if (field.isPoseWithinArea(botpose)) {
          SmartDashboard.putNumber("Robot Pose X", drivetrain.getState().Pose.getX());
          SmartDashboard.putNumber("Robot Pose Y", drivetrain.getState().Pose.getY());
          SmartDashboard.putNumber("Robot Pose 0", drivetrain.getState().Pose.getRotation().getDegrees());
          SmartDashboard.putNumber("Target Pose X", botpose.getX());
          SmartDashboard.putNumber("Target Pose Y", botpose.getY());
          SmartDashboard.putNumber("Target Pose 0", botpose.getRotation().getDegrees());
          SmartDashboard.putNumber("Pose Comparison", drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()));
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.getTranslation()) < 0.5
              || trust
              || LimelightHelpers.getRawFiducials(ll).length > 1) {
            drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(confidence, confidence, .01));
          } else {
            distanceError++;
            SmartDashboard.putNumber("Limelight Error", distanceError);
          }
        } else {
          fieldError++;
          SmartDashboard.putNumber("Field Error", fieldError);
        }
      } else {
        validError++;
        SmartDashboard.putNumber("Validity Error", validError);
      }
    } else {
      enableError++;
      SmartDashboard.putNumber("Enable Error", enableError);
    }
  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }

  public Pose2d tagPose() {
    if (LimelightHelpers.getRawFiducials(ll).length > 0) {
      Pose2d tagPose = LimelightHelpers.getTargetPose3d_RobotSpace(ll).toPose2d();
      double distance = Math.sqrt(Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2));
      if (distance < 3) {
        Transform2d tagTransform = new Transform2d(tagPose.getTranslation(), tagPose.getRotation());
        return drivetrain.getState().Pose.plus(tagTransform);
      }
    }
    return new Pose2d(-1, -1, new Rotation2d());
  }
}