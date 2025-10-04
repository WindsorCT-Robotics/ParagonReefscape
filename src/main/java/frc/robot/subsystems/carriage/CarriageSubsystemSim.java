package frc.robot.subsystems.carriage;

import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystemSim extends SubsystemBase {

    private final IntakeSimulation intakeSimulation;

    public CarriageSubsystemSim(AbstractDriveTrainSimulation driveTrain) {

        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Coral",
        // Specify the drivetrain to which this intake is attached
        driveTrain,
        // Width of the intake
        Meters.of(0.7),
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.2),
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        1);
    }

    @Override
    public void periodic() {
        
    }

    public void moveRollers(boolean runIntake) {
        if (runIntake) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }
    }

    @AutoLogOutput(key = "Sensor/OuttakeBeam")
    public boolean isCoralInside() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }

    public void scoreCoral(double velocity, double height) {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            SimulatedArena.getInstance()
            .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.4, 0),
            // Obtain robot speed from drive simulation
            MapleSimSwerveDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            MapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(height),
            // The initial speed of the coral
            MetersPerSecond.of(velocity),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-10)));
        } else {
            System.out.println("No coral");
        }
    }
}