package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Limelight;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TagAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private Pose2d tagPose;
    private Command pathfind;

    public TagAlignCommand(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(this.drivetrain, this.limelight);
    }

    @Override
    public void initialize() {
        tagPose = limelight.tagPose();
        if (tagPose.getX() == -1 && tagPose.getY() == -1) {
            this.cancel();
        }
        PathConstraints constraints = new PathConstraints(1, 1, 2, 1);
        pathfind = AutoBuilder.pathfindToPose(tagPose, constraints);
    }

    @Override
    public void execute() {
        pathfind.execute();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return pathfind.isFinished();
    }
}
