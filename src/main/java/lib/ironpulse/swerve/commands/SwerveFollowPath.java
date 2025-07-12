package lib.ironpulse.swerve.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.swerve.Swerve;
import lombok.Setter;

import java.util.function.Supplier;

public class SwerveFollowPath extends Command {
    private final Swerve swerve;
    private final PathPlannerPath path;
    @Setter
    private Supplier<Pose3d> poseWorldRobotSupplier;
    @Setter
    private Supplier<Pose3d> poseWorldTargetSupplier;
    @Setter
    private Distance translationTolerance;
    @Setter
    private Angle rotationTolerance;

    @Setter
    private PIDController translationController;
    @Setter
    private PIDController rotationController;
    @Setter
    private Strategy strategy = Strategy.PurePursuit;

    public SwerveFollowPath(Swerve swerve, PathPlannerPath path, PIDController translationController,
                            PIDController rotationController) {
        this.swerve = swerve;
        this.path = path;
        this.translationController = translationController;
        this.rotationController = rotationController;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    public enum Strategy {
        PurePursuit,
        AdaptivePurePursuit,
        RegulatedPurePursuit
    }
}
