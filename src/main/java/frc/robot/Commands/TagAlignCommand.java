package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TagAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private Pose2d tagPose;
    private Command pathfind;
    private static int counter1 = 0;
    private static int counter2 = 0;
    private static int counter3 = 0;

    public TagAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(this.drivetrain, this.limelight);
    }

    @Override
    public void initialize() {
        tagPose = limelight.tagPose();
        SmartDashboard.putNumber("Tag Pose X", tagPose.getX());
        SmartDashboard.putNumber("Tag Pose Y", tagPose.getY());
        SmartDashboard.putNumber("Tag Pose 0", tagPose.getRotation().getDegrees());
        if (tagPose.getX() == -1 && tagPose.getY() == -1) {
            this.cancel();
        }
        PathConstraints constraints = new PathConstraints(1, 1, 2, 1);
        pathfind = AutoBuilder.pathfindToPose(tagPose, constraints);
        counter1++;
        SmartDashboard.putNumber("Tag Align Command 1", counter1);
    }

    @Override
    public void execute() {
        pathfind.execute();
        counter2++;
        SmartDashboard.putNumber("Tag Align Command 2", counter2);
    }

    @Override
    public void end(boolean interrupted) {
        counter3++;
        SmartDashboard.putNumber("Tag Align Command 3", counter3);
    }

    @Override
    public boolean isFinished() {
        return pathfind.isFinished();
    }
}
