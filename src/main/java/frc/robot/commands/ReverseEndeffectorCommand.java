package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class ReverseEndeffectorCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final Timer timer = new Timer();

    public ReverseEndeffectorCommand(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
    }

    @Override
    public void initialize() {
        endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.REVERSE_POKE_VOLTAGE.getValue());
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setRollerVoltage(0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}