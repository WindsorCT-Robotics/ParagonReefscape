package frc.robot.hardware.impl.sim.carriage;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.impl.carriage.CarriageDifferentialMotors;
import frc.robot.hardware.sim.ISimDutyRPMMotor;
import frc.robot.hardware.sim.ISimHardware;

public class SimCarriage extends CarriageDifferentialMotors implements ISimHardware {
    private final ISimDutyRPMMotor rightMotor;
    private final ISimDutyRPMMotor leftMotor;

    public SimCarriage(ISimDutyRPMMotor rightMotor, ISimDutyRPMMotor leftMotor) {
        super(rightMotor, leftMotor);
        
        this.rightMotor = rightMotor;
        this.leftMotor = leftMotor;
    }

    @Override
    public void iterate(Voltage batteryVoltage, Time stepTime) {
        rightMotor.iterate(batteryVoltage, stepTime);
        leftMotor.iterate(batteryVoltage, stepTime);
    }
}
