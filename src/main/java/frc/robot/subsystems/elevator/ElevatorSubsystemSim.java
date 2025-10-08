package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import frc.robot.hardware.ISimPositionalMotor;

public class ElevatorSubsystemSim extends ElevatorSubsystem {
    private final ISimPositionalMotor motor;
    private final LoggedMechanism2d visualizedElevator;
    private final LoggedMechanismRoot2d elevatorRoot;
    @SuppressWarnings("unused")
    private final LoggedMechanismLigament2d elevatorLigament;

    public ElevatorSubsystemSim(ISimPositionalMotor motor) {
        super(motor);
        this.motor = motor;
        visualizedElevator = new LoggedMechanism2d(Units.inchesToMeters(0), Units.inchesToMeters(0));
        elevatorRoot = visualizedElevator.getRoot("Elevator", Units.inchesToMeters(0), Units.inchesToMeters(0));
        elevatorLigament = elevatorRoot.append(new LoggedMechanismLigament2d("Elevator", Units.inchesToMeters(50), 90));
    }
    
    @Override
    public void periodic() {
        super.periodic();
        motor.iterate();
    }
}