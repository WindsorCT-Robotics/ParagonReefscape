// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Limelight.LimelightHelpers;
import frc.lib.Limelight.RectanglePoseArea;
import frc.lib.Limelight.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  Alliance alliance;

  private String ll = "limelight";
  private Boolean enable = true;
  private Boolean trust = true;
  private Boolean invalidError = false;
  private Boolean fieldError = false;
  private Boolean distanceError = false;
  private PoseEstimate botpose;
  private static final RectanglePoseArea field =
        new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    HttpCamera httpCamera = new HttpCamera("limelight", "http://10.5.71.11:5800/");
    CameraServer.addCamera(httpCamera);
    httpCamera.setExposureAuto();
  }

  @Override
  public void periodic() { // Checking drivetrain data against limelight/apriltag data to confirm accuracy of robot position, and updates the position accordingly.
    if (enable) {
      // Double targetDistanceX = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getX();
      // Double targetDistanceY = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getY();
      // Double targetDistanceZ = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getZ();
      
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
      
      // LimelightHelpers.SetRobotOrientation(ll, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(ll, drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

      Double tx_output = LimelightHelpers.getTX(ll);
      Double ty_output = LimelightHelpers.getTY(ll);
      Double ta_output = LimelightHelpers.getTA(ll);

      Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(ll).getTranslation().getDistance(new Translation3d()); //Calculates how far away the april tag is
      Double confidence = (targetDistance - 1) / 6;

      if (!(tx_output == 0 && ty_output == 0 && ta_output == 0)) { //If result finds a vaild target then continues if statement result.valid
        invalidError = false;
        // botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
        botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
        SmartDashboard.putNumber("Number Of Apriltags", botpose.tagCount);
        if (field.isPoseWithinArea(botpose.pose)) { 
          fieldError = false;
          if (drivetrain.getState().Pose.getTranslation().getDistance(botpose.pose.getTranslation()) < 0.1 && Math.abs(drivetrain.getState().Pose.getRotation().getDegrees() - botpose.pose.getRotation().getDegrees()) < 3 //Compares the drivetrain aussumed position to the limelight's assumed position
              || trust
              || botpose.tagCount > 1) {
            distanceError = false;
            trust = false;
            drivetrain.addVisionMeasurement(
                botpose.pose,
                Utils.fpgaToCurrentTime(botpose.timestampSeconds), // Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) - (result.latency_pipeline / 1000.0),
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

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

  public void trustLL(boolean trust) {
    this.trust = trust;
  }

  public String getLimelightName() {
    return ll;
  }

  public int getTagCount() {
    try {
      // botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
      botpose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);
      return botpose.tagCount;
    } catch (Exception e){
      return 0;
    }
  }
}