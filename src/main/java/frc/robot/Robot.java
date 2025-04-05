// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.PhotonCamera;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private VisionSim visionSim;
  private PhotonCamera camera;
  private String cameraName = "Main";

  private final RobotContainer m_robotContainer;

  StructArrayPublisher<Pose3d> CoralPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Coral", Pose3d.struct)
      .publish();
    
    StructArrayPublisher<Pose3d> AlgaePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("Algae", Pose3d.struct)
      .publish();

  public Robot() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (isReal()) {
      m_robotContainer.logControllers();
    }
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
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
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
  public void testPeriodic() {}

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
