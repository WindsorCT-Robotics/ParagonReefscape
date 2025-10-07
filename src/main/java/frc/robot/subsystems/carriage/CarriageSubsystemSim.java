package frc.robot.subsystems.carriage;

import frc.robot.hardware.sim.ISimBeamBreak;
import frc.robot.hardware.sim.ISimDifferentialMotors;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;

public class CarriageSubsystemSim extends CarriageSubsystem {
    private final ISimDifferentialMotors motors;
    private final ISimBeamBreak beamBreak;
    private final IntakeSimulation intakeSimulation;
    private final AbstractDriveTrainSimulation driveTrain;
    private static final Distance INTAKE_WIDTH = Meters.of(0.7);
    private static final Distance INTAKE_LENGTH_FROM_ROBOT_WHEN_ACTIVE = Meters.of(0.2);
    private static final int INTAKE_CAPACITY = 1;

    public CarriageSubsystemSim(ISimDifferentialMotors motors, ISimBeamBreak beamBreak, AbstractDriveTrainSimulation driveTrain) {
        super(motors, beamBreak);

        this.motors = motors;
        this.beamBreak = beamBreak;
        this.driveTrain = driveTrain;

        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Coral",
        // Specify the drivetrain to which this intake is attached
        driveTrain,
        // Width of the intake
        INTAKE_WIDTH,
        // The extension length of the intake beyond the robot's frame (when activated)
        INTAKE_LENGTH_FROM_ROBOT_WHEN_ACTIVE,
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        INTAKE_CAPACITY);
    }

    @Override
    public void periodic() {
        super.periodic();
        motors.iterate();
        beamBreak.iterate();
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

    // TODO: This function should probably be private and the values should be calculated based on physics simulation
    public void scoreCoral(double velocity, double height) {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            SimulatedArena.getInstance()
            .addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveTrain.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.4, 0),
            // Obtain robot speed from drive simulation
            driveTrain.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveTrain.getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(height),
            // The initial speed of the coral
            // TODO: Calculate this based on motor physics simulation
            MetersPerSecond.of(velocity),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-10)));
        } else {
            System.out.println("No coral");
        }
    }
}