package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;

public class ShootCommand extends Command{
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ShootCommand(EndEffectorSubsystem endEffectorSubsystem, IndicatorSubsystem indicatorSubsystem){
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
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setRollerVoltage(0);
        indicatorSubsystem.setPattern(Patterns.NORMAL);
    }

}
