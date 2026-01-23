package frc.robot.hardware.impl.sim.carriage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.impl.carriage.CarriageMotor;
import frc.robot.hardware.sim.ISimHardware;

public class SimCarriageMotor extends CarriageMotor implements ISimHardware {
    private final SparkMaxSim motorSim;
    private static final int MOTOR_COUNT = 1;

    public SimCarriageMotor(SparkMax motor, boolean willRunInverted) {
        super(motor, willRunInverted);

        motorSim = new SparkMaxSim(motor, DCMotor.getNEO(MOTOR_COUNT));
    }

    @Override
    public void iterate(Voltage batteryVoltage, Time stepTime) {
        motorSim.iterate(motorSim.getVelocity(), batteryVoltage.in(Volts), stepTime.in(Seconds));
        Logger.recordOutput("CarriageMotor/Velocity", motorSim::getVelocity);
        Logger.recordOutput("CarriageMotor/Position", motorSim::getPosition);
        Logger.recordOutput("CarriageMotor/Voltage", motorSim::getBusVoltage);
    }
}
