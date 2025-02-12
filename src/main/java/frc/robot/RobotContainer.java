// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RightL2ScoreCommand;
import frc.robot.commands.RightL3ScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ExtendElevatorCommand;
import frc.robot.commands.IntakeBeamCommand;
import frc.robot.commands.LeftL2ScoreCommand;
import frc.robot.commands.LeftL3ScoreCommand;
import frc.robot.commands.OuttakeBeamCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.commands.ResetSimPoseToDriveCommand;
import frc.robot.commands.RetractElevatorCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
    private ElevatorSubsystem elevator;
    private CarriageSubsystem carriage;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric relativeDrive = new RobotCentric();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public final Limelight vision = new Limelight(drivetrain);

    Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));
    
    public RobotContainer() {
        elevator = new ElevatorSubsystem();
        carriage = new CarriageSubsystem();

        RegisterNamedComands();

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Autos'", autoChooser);

        configureBindings();
    }

    private void RegisterNamedComands()
    {
        NamedCommands.registerCommand("IntakeBeamCommand", new IntakeBeamCommand(carriage));
        NamedCommands.registerCommand("LeftL3ScoreCommand", new LeftL3ScoreCommand(carriage, elevator, drivetrain));
        NamedCommands.registerCommand("RightL3ScoreCommand", new RightL3ScoreCommand(carriage, elevator, drivetrain));
        NamedCommands.registerCommand("LeftL2ScoreCommand", new LeftL2ScoreCommand(carriage, elevator, drivetrain));
        NamedCommands.registerCommand("RightL2ScoreCommand", new RightL2ScoreCommand(carriage, elevator, drivetrain));
    }

    public void BuildAutos()
    {

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Math.abs(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Math.abs(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Math.abs(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.leftBumper().onTrue(new IntakeBeamCommand(carriage));
        joystick.rightBumper().onTrue(new OuttakeBeamCommand(carriage));

        // joystick.povUp().onTrue(new ExtendElevatorCommand(elevator));
        // joystick.povDown().onTrue(new RetractElevatorCommand(elevator));
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.75)));
        joystick.povDown().onTrue(new ReefAlignCommand(drivetrain, vision));

        // joystick.povLeft().onTrue(new LeftBeamAdjustment(drivetrain));
        // joystick.povRight().onTrue(new RightBeamAdjustment(drivetrain));
        joystick.povRight().onTrue(new ResetSimPoseToDriveCommand(drivetrain));

        joystick.a().onTrue(new LeftL2ScoreCommand(carriage, elevator, drivetrain));
        joystick.b().onTrue(new RightL2ScoreCommand(carriage, elevator, drivetrain));
        joystick.x().onTrue(new LeftL3ScoreCommand(carriage, elevator, drivetrain));
        joystick.y().onTrue(new RightL3ScoreCommand(carriage, elevator, drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
