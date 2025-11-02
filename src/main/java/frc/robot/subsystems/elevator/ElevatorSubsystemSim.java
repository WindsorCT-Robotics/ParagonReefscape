package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.sim.ISimPositionalMotor;

public class ElevatorSubsystemSim extends ElevatorSubsystem {
    private final ISimPositionalMotor motor;
    private final LoggedMechanism2d visualizedElevator;
    private final LoggedMechanismRoot2d elevatorRoot;
    @SuppressWarnings("unused")
    private final LoggedMechanismLigament2d elevatorLigament;

    private static final Distance VISUALIZED_ELEVATOR_LENGTH = Meters.of(0);
    private static final Distance VISUALIZED_ELEVATOR_WIDTH = Meters.of(0);
    private static final Distance ELEVATOR_ROOT_X = Meters.of(0);
    private static final Distance ELEVATOR_ROOT_Y = Meters.of(0);
    private static final Distance ELEVATOR_LIGAMENT_LENGTH = Inches.of(50);
    private static final Angle ELEVATOR_LIGAMENT_ANGLE = Degrees.of(90);

    public ElevatorSubsystemSim(String subsystemName, ISimPositionalMotor motor) {
        super(subsystemName, motor);
        this.motor = motor;
        visualizedElevator = new LoggedMechanism2d(VISUALIZED_ELEVATOR_WIDTH.in(Meters), VISUALIZED_ELEVATOR_LENGTH.in(Meters));
        elevatorRoot = visualizedElevator.getRoot("Elevator", ELEVATOR_ROOT_X.in(Meters), ELEVATOR_ROOT_Y.in(Meters));
        elevatorLigament = elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", ELEVATOR_LIGAMENT_LENGTH.in(Inches), ELEVATOR_LIGAMENT_ANGLE.in(Degrees)));
    }
    
    @Override
    public void periodic() {
        super.periodic();
        motor.iterate();
    }
}