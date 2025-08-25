package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

@Deprecated
public class ShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final Timer timer = new Timer();

    public ShootCommand(EndEffectorSubsystem endEffectorSubsystem, IndicatorSubsystem indicatorSubsystem) {
        this.indicatorSubsystem = indicatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(Patterns.SHOOT);
    }

    @Override
    public void execute() {
        endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE.getValue());
        if (!endEffectorSubsystem.hasCoral() && !timer.isRunning()) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setRollerVoltage(0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.3);
    }

}
