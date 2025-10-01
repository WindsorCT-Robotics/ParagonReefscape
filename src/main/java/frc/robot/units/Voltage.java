package frc.robot.units;

public class Voltage {
    private final double voltage;
    
    public Voltage(double voltage) {
        this.voltage = voltage;
    }

    public double asDouble() {
        return this.voltage;
    }
}
