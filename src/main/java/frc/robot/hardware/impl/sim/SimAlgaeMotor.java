package frc.robot.hardware.impl.sim;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.hardware.impl.AlgaeMotor;
import frc.robot.hardware.sim.ISimHardware;

public class SimAlgaeMotor extends AlgaeMotor implements ISimHardware {
    private final SparkMaxSim motor;
    public static final int MOTOR_COUNT = 1;

    public SimAlgaeMotor(SparkMax motor) {
        super(motor);
        this.motor = new SparkMaxSim(motor, DCMotor.getNEO(MOTOR_COUNT));
    }

    @Override
    public void iterate(Voltage batteryVoltage, Time stepTime) {
        motor.iterate(motor.getVelocity(), batteryVoltage.in(Volts), stepTime.in(Seconds));
        Logger.recordOutput("AlgaeMotor/Velocity", motor::getVelocity);
    }
}
