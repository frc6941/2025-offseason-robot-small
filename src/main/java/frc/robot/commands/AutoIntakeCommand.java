package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.swerve.Swerve;

public class AutoIntakeCommand extends ParallelCommandGroup {
    public AutoIntakeCommand(
            Swerve swerve,
            EndEffectorSubsystem endEffectorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IndicatorSubsystem indicatorSubsystem) {
        addCommands(
                new NavToStationCommand(swerve, indicatorSubsystem),
                new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem)
        );
    }

    public AutoIntakeCommand(
            Swerve swerve,
            EndEffectorSubsystem endEffectorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            boolean enableRumble) {
        addCommands(
                new NavToStationCommand(swerve, indicatorSubsystem),
                new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem, enableRumble)
        );
    }
}
