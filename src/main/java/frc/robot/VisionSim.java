/*
* MIT License
*
* Copyright (c) PhotonVision
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
    // Simulation
    private static PhotonCameraSim cameraSim;
        private VisionSystemSim visionSim;
        private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        private final double camPitch = Units.degreesToRadians(0.0);
        private final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));
    
        public VisionSim(PhotonCamera cam_in) {
            // ----- Simulation
            if (Robot.isSimulation()) {
                // Create the vision system simulation which handles cameras and targets on the field.
                visionSim = new VisionSystemSim("main");
                // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
                visionSim.addAprilTags(kTagLayout);
                // Create simulated camera properties. These can be set to mimic your actual camera.
                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(1920, 1080, Rotation2d.fromDegrees(90));
                cameraProp.setCalibError(0.35, 0.10);
                cameraProp.setFPS(70);  
                cameraProp.setAvgLatencyMs(0);
                cameraProp.setLatencyStdDevMs(0);
                // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
                // targets.
                cameraSim = new PhotonCameraSim(cam_in, cameraProp);
                // Add the simulated camera to view the targets on this simulated field.
                visionSim.addCamera(cameraSim, kRobotToCam);
    
                cameraSim.enableDrawWireframe(true);
            }
        }
    
        // ----- Simulation
    
        public static double getBestAprilTagSim() {
            if (cameraSim.getCamera().getLatestResult().hasTargets()) {
                System.out.println(cameraSim.getCamera().getLatestResult().getBestTarget().fiducialId);
            return cameraSim.getCamera().getLatestResult().getBestTarget().fiducialId;
        } else {
            System.out.println("No tags");
            return -1;
        }
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}