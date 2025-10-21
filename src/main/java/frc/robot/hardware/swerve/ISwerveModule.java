package frc.robot.hardware.swerve;

import frc.robot.hardware.IMotor;
import frc.robot.hardware.IGyro;

public interface ISwerveModule {
    IMotor DriveMotor;
    IMotor AzimuthMotor;
    IGyro Gyro;
}
