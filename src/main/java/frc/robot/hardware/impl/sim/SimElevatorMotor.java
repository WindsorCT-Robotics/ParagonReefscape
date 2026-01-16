package frc.robot.hardware.impl.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.impl.ElevatorMotor;
import frc.robot.hardware.sim.ISimHardware;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class SimElevatorMotor extends ElevatorMotor implements ISimHardware {
    private final SparkMaxSim elevSim;
    private static final int MOTOR_COUNT = 1;

    public SimElevatorMotor(SparkMax motor) {
        super(motor);
        elevSim = new SparkMaxSim(motor, DCMotor.getNEO(MOTOR_COUNT));
    }

    @Override
    public void iterate(Voltage batteryVoltage, Time stepTime) {
        elevSim.iterate(elevSim.getVelocity(), batteryVoltage.in(Volts), stepTime.in(Seconds));
        Logger.recordOutput("Elevator/Height", elevSim.getPosition());
        Logger.recordOutput("Elevator/Velocity", elevSim.getVelocity());
    }
}