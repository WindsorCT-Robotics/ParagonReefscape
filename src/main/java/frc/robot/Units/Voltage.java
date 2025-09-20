package frc.robot.Units;

public class Voltage {
    private final double voltage;
    
    public Voltage(double voltage) {
        this.voltage = voltage;
    }

    public double asDouble() {
        return this.voltage;
    }
}
