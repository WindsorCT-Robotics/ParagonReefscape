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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RightL2ScoreCommand;
import frc.robot.commands.RightL3ScoreCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ExtendElevatorCommand;
import frc.robot.commands.IntakeBeamCommand;
import frc.robot.commands.ManualIntakeOuttakeCommand;
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
    private final SwerveRequest.Idle coast = new SwerveRequest.Idle();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    private final CommandXboxController opController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public final Limelight vision = new Limelight(drivetrain);

    Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));
    
    public RobotContainer() {
        elevator = new ElevatorSubsystem();
        carriage = new CarriageSubsystem();

        RegisterNamedComands();

        autoChooser = AutoBuilder.buildAutoChooser("Optimal Path");
        SmartDashboard.putData("Autos", autoChooser);

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

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * Math.abs(driverController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftBumper().onTrue(new IntakeBeamCommand(carriage).until(opController.x()));
        // driverController.rightBumper().onTrue(new OuttakeBeamCommand(carriage));

        // driverController.povUp().onTrue(new ExtendElevatorCommand(elevator));
        // driverController.povDown().onTrue(new RetractElevatorCommand(elevator));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0.75)));
        
        driverController.leftStick().onTrue(new ReefAlignCommand(drivetrain, vision, opController));
        // driverController.povRight().onTrue(new ResetSimPoseToDriveCommand(drivetrain));

        driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed / 2) // Drive forward with negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed / 2) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * Math.abs(driverController.getRightX()) * MaxAngularRate / 1.5))); // Drive counterclockwise with negative X (left));
        
        driverController.povDown().and(driverController.x()).onTrue(new LeftL2ScoreCommand(carriage, elevator, drivetrain).until(opController.x()));
        driverController.povDown().and(driverController.b()).onTrue(new RightL2ScoreCommand(carriage, elevator, drivetrain).until(opController.x()));
        
        driverController.povUp().and(driverController.x()).onTrue(new LeftL3ScoreCommand(carriage, elevator, drivetrain).until(opController.x()));
        driverController.povUp().and(driverController.b()).onTrue(new RightL3ScoreCommand(carriage, elevator, drivetrain).until(opController.x()));


        opController.x().whileTrue(drivetrain.applyRequest(() -> coast));

        opController.povUp().onTrue(new ExtendElevatorCommand(elevator).until(opController.x()));
        opController.povDown().onTrue(new RetractElevatorCommand(elevator).until(opController.x()));

        opController.rightBumper().onTrue(new OuttakeBeamCommand(carriage).until(opController.x()));
        opController.leftBumper().onTrue(new IntakeBeamCommand(carriage).until(opController.x()));

        Trigger opLeftJoy = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.2);
        opLeftJoy.whileTrue(new ManualIntakeOuttakeCommand(carriage, () -> -opController.getRightY()));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
