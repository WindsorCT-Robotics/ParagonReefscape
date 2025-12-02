// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.TranslateAbsoluteCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.algae.AlgaeRemoverSubsystem;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.limelight.VisionSubsystem;

public class NewRobotContainer {
    public final CommandSwerveDrivetrain drivetrain;
    private VisionSubsystem vision;
    private ElevatorSubsystem elevator;
    private CarriageSubsystem carriage;
    private AlgaeRemoverSubsystem algaeRemover;
    private LEDSubsystem led;
    
    private final LinearVelocity MAX_SPEED;
    private final AngularVelocity MAX_ANGULAR_VELOCTIY;

    private final LinearVelocity VELOCITY_DEADBAND;
     private final AngularVelocity ANGULAR_VELOCITY_DEADBAND;
    
    private final SwerveRequest.FieldCentric swerveControllerFieldCentric;

    private final SwerveRequest.RobotCentric swerveControllerRobotCentric;

    private final Telemetry logger;

    // TODO: Unsure if autonomousChooser works.
    private final SendableChooser<Command> autonomousChooser;

    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    public NewRobotContainer() {
        // TODO: Assign Subsystems Names and Motors.
        // TODO: Allow for dynamically instantiating based on wether we're in sim mode or not
        drivetrain = TunerConstants.createDrivetrain();
        vision = new VisionSubsystem(null, null);
        elevator = new ElevatorSubsystem(null, null);
        carriage = new CarriageSubsystem(null, null, null);
        algaeRemover = new AlgaeRemoverSubsystem(null, null);
        led = new LEDSubsystem(null, null);

        autonomousChooser = AutoBuilder.buildAutoChooser("L3 Left");
    
        MAX_SPEED = drivetrain.getMaxVelocity();
        MAX_ANGULAR_VELOCTIY = drivetrain.getMaxAngularRate();

        VELOCITY_DEADBAND = MAX_SPEED.times(Percent.of(10).in(Value)); // TODO: Verify if this value is 10% of MAX_SPEED.
        ANGULAR_VELOCITY_DEADBAND = MAX_ANGULAR_VELOCTIY.times(Percent.of(10).in(Value)); // TODO: Verify if this value is 10% of MAX_ANGULAR_VELOCITY.
        
        swerveControllerFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(VELOCITY_DEADBAND.in(MetersPerSecond))
            .withRotationalDeadband(ANGULAR_VELOCITY_DEADBAND.in(RadiansPerSecond))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        swerveControllerRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(VELOCITY_DEADBAND.in(MetersPerSecond))
            .withRotationalDeadband(ANGULAR_VELOCITY_DEADBAND.in(RadiansPerSecond))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        logger = new Telemetry(MAX_SPEED.in(MetersPerSecond));

        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);


        assignControllerBindings();
    }


    private void assignControllerBindings() {

        // Driver Controller will move robot field relative on default.
        drivetrain.setDefaultCommand(new TranslateAbsoluteCommand(
            drivetrain, 
            () -> Percent.of(driverController.getLeftX() * 100), 
            () -> Percent.of(driverController.getLeftY() * 100),
            () -> Percent.of(driverController.getRightX() * 100))
        );


    }

    private void registerPathplannerCommands() {

    }
}