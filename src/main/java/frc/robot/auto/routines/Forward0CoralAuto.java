package frc.robot.auto.routines;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import lib.ironpulse.utils.Logging;

public class Forward0CoralAuto extends AutoRoutine {
    private static PathPlannerPath testPath;

    public Forward0CoralAuto() {
        super("Forward0CoralAuto");
        try {
            testPath = PathPlannerPath.fromPathFile("Forward");
        } catch (Exception e) {
            Logging.error("Auto/Forward0CoralAuto", "Failed to load auto! %s", e.getMessage());
        }

    }

    @Override
    public Command getAutoCommand() {
        return Commands.parallel(
                AutoActions.followPath(testPath)
        );
    }

    @Override
    public Command getOnSelectCommand() {
        return AutoActions.resetOnPathStart(testPath);
    }
}
