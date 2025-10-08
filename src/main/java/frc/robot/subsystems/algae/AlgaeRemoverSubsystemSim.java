package frc.robot.subsystems.algae;

import frc.robot.hardware.ISimSpeedMotor;

public class AlgaeRemoverSubsystemSim extends AlgaeRemoverSubsystem {
    private final ISimSpeedMotor motor;

    public AlgaeRemoverSubsystemSim(ISimSpeedMotor motor) {
        super(motor);
        this.motor = motor;
    }
}