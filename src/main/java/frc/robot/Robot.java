// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.photonvision.PhotonCamera;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private VisionSim visionSim;
  private PhotonCamera camera;
  private String cameraName = "Main";

  private int r;
  private int g;
  private int b;

  private final RobotContainer m_robotContainer;

  StructArrayPublisher<Pose3d> CoralPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Coral", Pose3d.struct)
      .publish();
    
    StructArrayPublisher<Pose3d> AlgaePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Algae", Pose3d.struct)
      .publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.led.checkAprilTags(m_robotContainer.vision);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    for (double t = 0.0; t <= 1.0; t += 0.01) {
      // Map t (0 to 1) to a specific range of colors in the rainbow
      int r = (int) (255 * (1 - Math.abs(t * 6 - 3) - Math.abs(t * 6 - 4)));
      int g = (int) (255 * (1 - Math.abs(t * 6 - 2) - Math.abs(t * 6 - 3)));
      int b = (int) (255 * (1 - Math.abs(t * 6 - 1) - Math.abs(t * 6 - 2)));

      try {
        Thread.sleep((long) 100.0);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      m_robotContainer.led.setLedColorAll(r, g, b);
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.led.setLedColorAll(0, 0, 0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    for (double t = 0.0; t <= 1.0; t += 0.01) {
        // Map t (0 to 1) to a smooth rainbow gradient
        int r = (int) (Math.abs(Math.sin(t * Math.PI * 2)) * 255);
        int g = (int) (Math.abs(Math.sin((t + 1.0 / 3.0) * Math.PI * 2)) * 255);
        int b = (int) (Math.abs(Math.sin((t + 2.0 / 3.0) * Math.PI * 2)) * 255);
        
        // Sleep for a short period to visualize the effect
        try {
            Thread.sleep(50);  // Adjust sleep time for desired speed of transition
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Set the LED color
        m_robotContainer.led.setLedColorAll(r, g, b);
        SmartDashboard.putNumber("R", r);
        SmartDashboard.putNumber("G", g);
        SmartDashboard.putNumber("B", b);
    }
  }


  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    // Obtains the default instance of the simulation world, which is a Reefscape Arena.
    SimulatedArena.getInstance();
    for (int i = 0; i < 10; i++) {
      SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(5, 5, Rotation2d.fromDegrees(0))));
      SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(5, 5)));
      SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(5, 5)));
    }

    camera = new PhotonCamera(cameraName);

    visionSim = new VisionSim(camera);
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    visionSim.simulationPeriodic(MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    DogLog.log("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    DogLog.log("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public RobotContainer getRobotContainer() {
    return m_robotContainer;
  }
}
