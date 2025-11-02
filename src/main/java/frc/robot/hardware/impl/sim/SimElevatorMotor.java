package frc.robot.hardware.impl.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.impl.ElevatorMotor;
import frc.robot.hardware.sim.ISimMotor;

import org.littletonrobotics.junction.Logger;


public class SimElevatorMotor extends ElevatorMotor implements ISimMotor {
    private final SparkMaxSim elevSim;

    public SimElevatorMotor(SparkMax motor) {
        super(motor);
        elevSim = new SparkMaxSim(elevMotor, DCMotor.getNEO(1));
    }

    @Override
    public void iterate() {
        elevSim.iterate(elevSim.getVelocity(), 12, 1);
        Logger.recordOutput("Height", elevSim.getPosition());
    }
}