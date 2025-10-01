package frc.robot.units;

public class RotationsPerMinute {
    private final double rotations;
    private final int minutes;

    public RotationsPerMinute(double rotations, int minutes) {
        this.rotations = rotations;
        this.minutes = minutes;
    }

    public double asDouble() { return rotations / minutes; }
}