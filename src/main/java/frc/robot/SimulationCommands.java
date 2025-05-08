package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

public class SimulationCommands {
    public SimulationCommands() {

    }

    public void spawnCoral() {
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getX(), MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getY(), Rotation2d.fromDegrees(0))));
    }

    public void spawnAlgae() {
        SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getX(), MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getY())));
    }

    public void spawnAlgaeOnCoral() {
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getX(), MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getY())));
    }

    public void autoSpawnCoral() {

    }
}
