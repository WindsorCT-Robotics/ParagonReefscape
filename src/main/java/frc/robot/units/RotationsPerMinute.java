package frc.robot.units;

public class RotationsPerMinute {
    private final double rpm;

    public RotationsPerMinute(double rotations, int minutes) {
        rpm =  rotations / minutes;
    }

    public RotationsPerMinute(double rpm) {
        this.rpm = rpm;
    }

    public double asDouble() { return rpm; }
}