package frc.robot.hardware.impl.sim.carriage;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.impl.carriage.CarriageLeftMotor;
import frc.robot.hardware.sim.ISimHardware;

public class SimCarriageLeftMotor extends CarriageLeftMotor implements ISimHardware {
    private final SparkMaxSim motorSim;
    private static final int MOTOR_COUNT = 1;

    public SimCarriageLeftMotor(SparkMax motor) {
        super(motor);

        motorSim = new SparkMaxSim(motor, DCMotor.getNEO(MOTOR_COUNT));
    }

    @Override
    public void iterate(Voltage batteryVoltage, Time stepTime) {
        motorSim.iterate(motorSim.getVelocity(), batteryVoltage.in(Volts), stepTime.in(Seconds));
        Logger.recordOutput("CarriageLeftMotor/Velocity", motorSim::getVelocity);
        Logger.recordOutput("CarriageLeftMotor/Position", motorSim::getPosition);
        Logger.recordOutput("CarriageLeftMotor/Voltage", motorSim::getBusVoltage);
    }
}
