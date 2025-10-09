package frc.robot.subsystems.algae;

import frc.robot.hardware.ISimSpeedMotor;

public class AlgaeRemoverSubsystemSim extends AlgaeRemoverSubsystem {
    private final ISimSpeedMotor motor;

    public AlgaeRemoverSubsystemSim(String subsystemName, ISimSpeedMotor motor) {
        super(subsystemName, motor);
        this.motor = motor;
    }
}