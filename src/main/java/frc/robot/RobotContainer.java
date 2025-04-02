// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.*;
import frc.robot.subsystems.AlgaeRemoverSubsystem;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.Limelight;

public class RobotContainer {
    private ElevatorSubsystem elevator;
    private CarriageSubsystem carriage;
    private AlgaeRemoverSubsystem algaeRemover;
    public LEDSubsystem led;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    @SuppressWarnings("unused")
    private final SwerveRequest.RobotCentric relativeDrive = new RobotCentric();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @SuppressWarnings("unused")
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
    @SuppressWarnings("unused")
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    @SuppressWarnings("unused")
    private final SwerveRequest.Idle coast = new SwerveRequest.Idle();
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    private final CommandXboxController opController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public final Limelight vision = new Limelight(drivetrain);

    Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(1, 0));
    
    public RobotContainer() {
        elevator = new ElevatorSubsystem();
        carriage = new CarriageSubsystem();
        algaeRemover = new AlgaeRemoverSubsystem();
        led = new LEDSubsystem();

        RegisterNamedComands();

        autoChooser = AutoBuilder.buildAutoChooser("L2 Left Inner Barge");
        SmartDashboard.putData("Autos", autoChooser);

        configureBindings();
    }

    private void RegisterNamedComands()
    {
        NamedCommands.registerCommand("IntakeBeamCommand", new CoralIntakeCommand(carriage));

        NamedCommands.registerCommand("LeftScoreCommand", new ScoreNoElevatorCommand(carriage, drivetrain, "left"));
        NamedCommands.registerCommand("RightScoreCommand", new ScoreNoElevatorCommand(carriage, drivetrain, "right"));

        NamedCommands.registerCommand("LeftL3ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "left", 3.0));
        NamedCommands.registerCommand("RightL3ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "right", 3.0));
        NamedCommands.registerCommand("LeftL2ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "left", 2.0));
        NamedCommands.registerCommand("RightL2ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "right", 2.0));
        NamedCommands.registerCommand("LeftL1ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "left", 1.0));
        NamedCommands.registerCommand("RightL1ScoreCommand", new ScoreCommand(carriage, elevator, drivetrain, "right", 1.0));

        NamedCommands.registerCommand("AlgaePick", new AlgaeMoveCommand(algaeRemover, true, 0.7));

        NamedCommands.registerCommand("L1", new ElevatorMoveCommand(elevator, 1.0));
        NamedCommands.registerCommand("L2", new ElevatorMoveCommand(elevator, 2.0));
        NamedCommands.registerCommand("L2_5", new ElevatorMoveCommand(elevator, 2.5));
        NamedCommands.registerCommand("L3", new ElevatorMoveCommand(elevator, 3.0));

        NamedCommands.registerCommand("MoveForwardRelative", new MoveDrivetrainCommand(drivetrain, 0.2, 0, true));
        NamedCommands.registerCommand("MoveForwardRelative 0.4", new MoveDrivetrainCommand(drivetrain, 0.4, 0, true));



        NamedCommands.registerCommand("MoveBackwardsRelative", new MoveDrivetrainCommand(drivetrain, 0.2, 180, true));
        NamedCommands.registerCommand("MoveBackwardsRelative 0.4", new MoveDrivetrainCommand(drivetrain, 0.4, 180, true));
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

        elevator.setDefaultCommand(new ElevatorControlCommand(elevator, 1));
        algaeRemover.setDefaultCommand(new AlgaeMoveCommand(algaeRemover, true, 0.4));

        Trigger opLeftTrigger = new Trigger(() -> opController.getLeftTriggerAxis() > 0.2 || opController.getLeftTriggerAxis() < -0.2);
        Trigger opRightTrigger = new Trigger(() -> opController.getRightTriggerAxis() > 0.2 || opController.getRightTriggerAxis() < -0.2);
        Trigger opRightJoy = new Trigger(() -> opController.getRightY() > 0.2 || opController.getRightY() < -0.2);

        Trigger driverLeftJoy = new Trigger(() -> driverController.getLeftY() > 0.1 || driverController.getLeftY() < -0.1);
        Trigger driverRightJoy = new Trigger(() -> driverController.getRightX() > 0.1 || driverController.getRightX() < -0.1);

        Trigger isValidTarget = new Trigger(() -> drivetrain.isValidTarget(vision));

        Trigger opLock = new Trigger(() -> (driverLeftJoy.getAsBoolean() || driverRightJoy.getAsBoolean() || !isValidTarget.getAsBoolean()));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        
        // Driver Bindings
        // Intake
        driverController.leftBumper().onTrue(new CoralIntakeCommand(carriage).until(driverController.x()));

        // Relative Drive Forward
        driverController.a().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed / 2).withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed / 2)));
        
        // Auto Reef Alignment
        driverController.leftStick().onTrue(drivetrain.pathToAlign(vision, false, "center"));
        driverController.b().and(driverController.leftStick()).onTrue(drivetrain.pathToAlign(vision, false, "right"));
        driverController.x().and(driverController.leftStick()).onTrue(drivetrain.pathToAlign(vision, false, "left"));

        // Auto Coral Station Alignment
        driverController.y().and(driverController.leftStick()).onTrue(drivetrain.pathToAlign(vision, true, "center").until(driverController.x()));

        // Auto direction align to coral stations
        driverController.y().toggleOnTrue(drivetrain.setOrientation(driverController, vision, true)
        .until(driverRightJoy)
        .until(driverController.b())
        .until(opController.x())
        .until(opController.b())
        .until(opLeftTrigger)
        .until(opRightTrigger)
        .until(opController.leftBumper())
        .until(opController.rightBumper()));

        //  Auto direction align to reef
        driverController.b().toggleOnTrue(drivetrain.setOrientation(driverController, vision, false)
        .until(driverController.y())
        .until(opController.x())
        .until(opController.b())
        .until(opLeftTrigger)
        .until(opRightTrigger)
        .until(opController.leftBumper())
        .until(opController.rightBumper()));

        // reset sim pose
        driverController.povRight().onTrue(new ResetSimPoseToDriveCommand(drivetrain));

        // Reduces max speed by a factor of 2
        driverController.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed / 2) // Drive forward with negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed / 2) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * Math.abs(driverController.getRightX()) * MaxAngularRate / 1.5))); // Drive counterclockwise with negative X (left));

        // reposition coral in carriage
        driverController.start().onTrue(new RepositionCoralCommand(carriage));

        // reset pigeon 2 yaw to 0
        driverController.start().and(driverController.back()).onTrue(new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0.0)));

        // Operator Bindings

        // Trough score
        opController.x().onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "left", 1.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));
        opController.b().onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "right", 1.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));

        // Manual
        opController.x().and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "left", 1.0));
        opController.b().and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "right", 1.0));

        // L2 score
        opRightTrigger.onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "right", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));
        opLeftTrigger.onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "left", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));

        // Manual
        opRightTrigger.and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "right", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));
        opLeftTrigger.and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "left", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));

        // Lower algae remove + score L2
        opRightTrigger.and(opController.back()).onTrue(new PathScoreAlgaeCommand(carriage, elevator, drivetrain, vision, "right", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()).unless(opLock));
        opLeftTrigger.and(opController.back()).onTrue(new PathScoreAlgaeCommand(carriage, elevator, drivetrain, vision, "left", 2.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()).unless(opLock));

        // L3 score
        opController.leftBumper().onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "left", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));
        opController.rightBumper().onTrue(new PathScoreCommand(carriage, elevator, drivetrain, vision, "right", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));

        // Manual
        opController.leftBumper().and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "left", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));
        opController.rightBumper().and(opController.start()).onTrue(new ScoreCommand(carriage, elevator, drivetrain, "right", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, false, 0.4)).until(opController.leftStick()).unless(opLock));

        // Upper algae remove + score L3
        opController.leftBumper().and(opController.back()).onTrue(new PathScoreAlgaeCommand(carriage, elevator, drivetrain, vision, "left", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()).unless(opLock));
        opController.rightBumper().and(opController.back()).onTrue(new PathScoreAlgaeCommand(carriage, elevator, drivetrain, vision, "right", 3.0).deadlineWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()).unless(opLock));

        // Extends and retracts the elevator
        opController.povUp().toggleOnTrue(new ElevatorControlCommand(elevator, 3).until(opController.leftStick()));
        opController.povRight().toggleOnTrue(new ElevatorControlCommand(elevator, 2.5).until(opController.leftStick()));
        opController.povLeft().toggleOnTrue(new ElevatorControlCommand(elevator, 2).until(opController.leftStick()));
        opController.povDown().toggleOnTrue(new ElevatorControlCommand(elevator, 1).until(opController.leftStick()));

        // Resets elevator
        opController.rightStick().onTrue(new ElevatorResetCommand(elevator));

        // Turn on algae remover and move elevator for lower algae
        opController.back().and(opController.a()).toggleOnTrue(new ElevatorControlCommand(elevator, 1.0).alongWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()));
        
        // Turn on algae remover and move elevator for upper algae
        opController.back().and(opController.y()).toggleOnTrue(new ElevatorControlCommand(elevator, 2.5).alongWith(new AlgaeMoveCommand(algaeRemover, true, 0.7)).until(opController.leftStick()));

        // Manually controls the intake and outtake rollers
        opRightJoy.and(opController.start()).whileTrue(new ManualMoveRollersCommand(carriage, () -> -opController.getLeftY()));
        opController.back().and(opController.start()).onTrue(new CoralOuttakeCommand(carriage, 2.0, "center"));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
