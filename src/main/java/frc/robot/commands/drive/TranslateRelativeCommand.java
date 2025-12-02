package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TranslateRelativeCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LinearVelocity MaxVelocity;
    private final AngularVelocity MaxAngularRate;
    private RobotCentric robotCentric;
    private Supplier<Dimensionless> xLeftAxis;
    private Supplier<Dimensionless> yLeftAxis;
    private Supplier<Dimensionless> xRightAxis;

    public TranslateRelativeCommand(
        CommandSwerveDrivetrain drivetrain, 
        Supplier<Dimensionless> xLeftAxis, 
        Supplier<Dimensionless> yLeftAxis,
        Supplier<Dimensionless> xRightAxis) {
        this.drivetrain = drivetrain;
        this.MaxVelocity = drivetrain.getMaxVelocity();
        this.MaxAngularRate = drivetrain.getMaxAngularRate();
        this.xLeftAxis = xLeftAxis;
        this.yLeftAxis = yLeftAxis;
        this.xRightAxis = xRightAxis;
        

        robotCentric = new RobotCentric()
            .withDeadband(MaxVelocity)
            .withRotationalDeadband(MaxAngularRate);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        drivetrain.setControl(driveRequest(xLeftAxis, yLeftAxis, xRightAxis));
    }

    private RobotCentric driveRequest(
        Supplier<Dimensionless> xLeftAxis, Supplier<Dimensionless> yLeftAxis, 
        Supplier<Dimensionless> xRightAxis
        ) {
        return robotCentric
            .withVelocityX(yLeftAxis.get().in(Value) * MaxVelocity.in(MetersPerSecond)) // Relative to a driver station inverting the axis makes sense.
            .withVelocityY(xLeftAxis.get().in(Value) * MaxVelocity.in(MetersPerSecond))
            .withRotationalRate(xRightAxis.get().in(Value) * MaxAngularRate.in(RadiansPerSecond));
    }
}