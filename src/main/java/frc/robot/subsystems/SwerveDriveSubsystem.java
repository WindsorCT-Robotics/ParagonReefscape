package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.units.MetersPerSecond;
import frc.robot.units.Radians;
import frc.robot.units.RadiansPerSecond;

public class SwerveDriveSubsystem extends SubsystemBase {
    public SwerveDriveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        
    }
    
    /**
     * Moves and rotates the robot relative to the angle of the robot.
     * @param longditudinalSpeed The speed of the robot on the X-axis relative to the rotation of the robot in meters per second.
     * @param lateralSpeed The speed of the robot on the Y-axis relative to the rotation of the robot in meters per second.
     * @param rotationalSpeed The turning speed of the robot per second in radians.
     */
    public void robotOrientedDrive(MetersPerSecond longditudinalSpeed, MetersPerSecond lateralSpeed, RadiansPerSecond rotationalSpeed)
    {

    }

    /**
     * Moves and rotates the robot relative to the inital start-up angle of the robot.
     * @param wallSpeed The speed of the robot on the X-axis relative to the inital start-up of the robot in meters per second.
     * @param allianceSpeed The speed of the robot on the Y-axis relative to the inital start-up of the robot in meters per second.
     * @param rotationalSpeed The turning speed of the robot per second in radians.
     * @param currentAngle The current angle of the robot in radians.
     */
    public void fieldOrientedDrive(MetersPerSecond allianceSpeed, MetersPerSecond wallSpeed, RadiansPerSecond rotationalSpeed, Radians currentAngle)
    {

    }

    /**
     * Rotates the robot relative to a point in an xy plane.
     * @param centerOfRotation A point on an xy plane (Meters) that the robot will try to look at.
     * @param rotationSpeed The turning speed of the robot per second in radians.
     */
    public void rotateAround(Translation2d centerOfRotation, RadiansPerSecond rotationSpeed)
    {
        
    }
}
