package frc.robot.commands.composite;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.carriage.RepositionCoralCommand;
import frc.robot.commands.carriage.ScoreCoralCommand;
import frc.robot.commands.drive.pathing.DynamicPathfindToCommand;
import frc.robot.commands.elevator.PositionElevatorCommand;
import frc.robot.hardware.MotorDirection;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.carriage.CarriageSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.Position;

public class PathToScoreCommand extends SequentialCommandGroup {
    public PathToScoreCommand(
        CommandSwerveDrivetrain drivetrain, 
        ElevatorSubsystem elevator,
        Position elevatorPosition, 
        CarriageSubsystem carriage, 
        List<Waypoint> waypoints, 
        Rotation2d endHeading, 
        PathConstraints constraints) {
        addCommands(
            new ParallelCommandGroup(
                new RepositionCoralCommand(carriage),
                new SequentialCommandGroup(
                    new DynamicPathfindToCommand(drivetrain, waypoints, endHeading, constraints)
                    // ADD THE COMMAND BEAM ADJUSTMENT WHEN CREATED.
                ),
                new PositionElevatorCommand(elevator, elevatorPosition)
            ),
            new ScoreCoralCommand(carriage, MotorDirection.FORWARD)
        );
    }
}
