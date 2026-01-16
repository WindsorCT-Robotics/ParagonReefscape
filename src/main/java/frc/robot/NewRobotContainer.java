// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.algae.PushAlgaeCommand;
import frc.robot.commands.carriage.RepositionCoralCommand;
import frc.robot.commands.drive.AngleToCoralStationCommand;
import frc.robot.commands.drive.SetAngleCommand;
import frc.robot.commands.drive.TranslateFromControllerAbsoluteCommand;
import frc.robot.commands.elevator.PositionElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.algae.AlgaeRemoverSubsystem;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;
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
        
        // TODO: These swerveControllers may not be required since the drive commmands do the same thing or controller drive commands are unessesary.
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

        setDefaultCommands();
        assignControllerBindings();
        registerPathplannerCommands();
    }


    private void assignControllerBindings() {
        assignDriverControllerBindings();
        assignOperatorControllerBindings();
    }

    private void assignDriverControllerBindings() {
        Trigger LeftTrigger = new Trigger(() -> driverController.leftTrigger(0.2).getAsBoolean());
        Trigger rightTrigger = new Trigger(() -> driverController.rightTrigger(0.2).getAsBoolean());

        Trigger leftJoystick = new Trigger(() -> driverController.getLeftY() > 0.1 || driverController.getLeftY() < -0.1);
        Trigger rightJoystick = new Trigger(() -> driverController.getRightX() > 0.1 || driverController.getRightX() < -0.1);

        driverController.leftBumper().onTrue(
            new LoadCoral(carriage).until(driverController.x())
        );

        // Relative Drive Forward
        driverController.a().whileTrue(
            new TranslateFromControllerRelativeCommand(
                drivetrain, 
                () -> curveAxisByExponent(Percent.of(driverController.getLeftX() * 100), SQUARED_INPUT),
                () -> curveAxisByExponent(Percent.of(driverController.getLeftY() * 100), SQUARED_INPUT), 
                () -> curveAxisByExponent(Percent.of(driverController.getRightX() * 100), SQUARED_INPUT),
                VELOCITY_DEADBAND,
                ANGULAR_VELOCITY_DEADBAND
            )
        );

        // Forces only 2 cardinal directions unless any until is triggered.
        driverController.y().toggleOnTrue(new AngleToCoralStationCommand(drivetrain, () -> drivetrain.getState().Pose.getY()) // TODO: Confirm that Y is width.
        .until(driverRightJoy)
        .until(driverController.b())
        .until(opController.x())
        .until(opController.b())
        .until(opLeftTrigger)
        .until(opRightTrigger)
        .until(opController.leftBumper())
        .until(opController.rightBumper()));

        // Forces only 6 cardinal directions unless any until is triggered.
        driverController.b().toggleOnTrue(
        .until(driverController.y())
        .until(opController.x())
        .until(opController.b())
        .until(opLeftTrigger)
        .until(opRightTrigger)
        .until(opController.leftBumper())
        .until(opController.rightBumper()));

        // Reduces max speed by a factor of 2
        driverController.rightBumper().whileTrue();

        // reposition coral in carriage
        driverController.start().onTrue();

        // reset pigeon 2 yaw to 0
        driverController.start().and(driverController.back()).onTrue(new InstantCommand(() -> drivetrain.getPigeon2().setYaw(0.0)));
    }

    private void assignOperatorControllerBindings() {
        
    }

    private void setDefaultCommands() {
        // Driver Controller will move robot field relative on default.
        drivetrain.setDefaultCommand(new TranslateFromControllerAbsoluteCommand(
            drivetrain, 
            () -> Percent.of(driverController.getLeftX() * 100), 
            () -> Percent.of(driverController.getLeftY() * 100),
            () -> Percent.of(driverController.getRightX() * 100),
            VELOCITY_DEADBAND,
            ANGULAR_VELOCITY_DEADBAND
            )
        );

        elevator.setDefaultCommand(new PositionElevatorCommand(elevator, Position.LEVEL_1));

        algaeRemover.setDefaultCommand(new PushAlgaeCommand(algaeRemover)); // TODO: What speed and direction should this be facing?
    }

    private void registerPathplannerCommands() {
        // Drive
        NamedCommands.registerCommand("IntakeBeamCommand", );

        // Elevator


        // Carriage


        // AlgaeRemover

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
        NamedCommands.registerCommand("AlgaePickReverse", new AlgaeMoveCommand(algaeRemover, false, 0.7));

        NamedCommands.registerCommand("L1", new ElevatorMoveCommand(elevator, 1.0));
        NamedCommands.registerCommand("L2", new ElevatorMoveCommand(elevator, 2.0));
        NamedCommands.registerCommand("L2_5", new ElevatorMoveCommand(elevator, 2.5));
        NamedCommands.registerCommand("L3", new ElevatorMoveCommand(elevator, 3.0));

        NamedCommands.registerCommand("MoveForwardRelative", new MoveDrivetrainCommand(drivetrain, 0.2, 0, true));
        NamedCommands.registerCommand("MoveForwardRelative 0.4", new MoveDrivetrainCommand(drivetrain, 0.4, 0, true));



        NamedCommands.registerCommand("MoveBackwardsRelative", new MoveDrivetrainCommand(drivetrain, 0.2, 180, true));
        NamedCommands.registerCommand("MoveBackwardsRelative 0.4", new MoveDrivetrainCommand(drivetrain, 0.4, 180, true));
    }
}