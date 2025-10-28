package frc.robot.hardware.impl.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import frc.robot.hardware.impl.ElevatorMotor;
import frc.robot.hardware.sim.ISimMotor;
import frc.robot.units.GearRatio;

import org.dyn4j.geometry.Mass;
import org.littletonrobotics.junction.Logger;


public class SimElevatorMotor extends ElevatorMotor implements ISimMotor {
    private final SparkMaxSim elevSim = new SparkMaxSim(elevMotor, DCMotor.getNEO(1));

    public SimElevatorMotor(SparkMax motor, GearRatio gearRatio, Angle pulleyCircumference, Mass elevatorWeight) {
        super(motor, gearRatio, pulleyCircumference, elevatorWeight);
    }

    @Override
    public void iterate() {
        elevSim.iterate(elevSim.getVelocity(), 12, 1);
        Logger.recordOutput("Height", elevSim.getPosition());
    }
}