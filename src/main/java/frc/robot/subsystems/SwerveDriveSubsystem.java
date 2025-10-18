package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.units.MetersPerSecond;
import frc.robot.units.Radians;
import frc.robot.units.RadiansPerSecond;

public class SwerveDriveSubsystem extends SubsystemBase {
    public SwerveDriveSubsystem() {

    }
    
    public void robotOrientedDrive(MetersPerSecond longditudinalSpeed, MetersPerSecond lateralSpeed, RadiansPerSecond rotationalSpeed)
    {

    }

    public void fieldOrientedDrive(MetersPerSecond allianceSpeed, MetersPerSecond wallSpeed, RadiansPerSecond rotationalSpeed, Radians currentAngle)
    {

    }

    public void rotateAround(Translation2d centerOfRotation, RadiansPerSecond rotationSpeed)
    {
        
    }
}
