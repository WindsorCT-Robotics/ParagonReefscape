package frc.robot.hardware.sim;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.ElevatorMotor;
import frc.robot.units.GearRatio;
import frc.robot.units.Kilograms;
import frc.robot.units.Radians;
import org.littletonrobotics.junction.Logger;

public class SimElevatorMotor extends ElevatorMotor {
    private final SparkMaxSim elevSim = new SparkMaxSim(elevMotor, DCMotor.getNEO(1));

    public SimElevatorMotor(SparkMax motor, GearRatio gearRatio, Radians pulleyCircumference, Kilograms elevatorWeight) {
        super(motor, gearRatio, pulleyCircumference, elevatorWeight);
    }

    public void update() {
        elevSim.iterate(elevSim.getVelocity(), 12, 1);
        Logger.recordOutput("Height", elevSim.getPosition());
    }
}