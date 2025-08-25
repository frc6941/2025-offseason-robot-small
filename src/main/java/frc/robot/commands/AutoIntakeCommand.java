package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.swerve.Swerve;

import java.util.Timer;

import static edu.wpi.first.units.Units.Seconds;

public class AutoIntakeCommand extends SequentialCommandGroup {
    public AutoIntakeCommand(
            Swerve swerve,
            EndEffectorSubsystem endEffectorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Superstructure superstructure) {
        addCommands(
                superstructure.runGoal(SuperstructureState.IDLE).until(elevatorSubsystem::isAtGoal),
                new NavToStationCommand(swerve, indicatorSubsystem),
                superstructure.runGoal(()-> SuperstructureState.INTAKE).until(endEffectorSubsystem::isFrontEE)
        );
    }

    public AutoIntakeCommand(
            Swerve swerve,
            EndEffectorSubsystem endEffectorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            Superstructure superstructure,
            boolean enableRumble) {
        addCommands(
                new NavToStationCommand(swerve, indicatorSubsystem),
                superstructure.runGoal(()->SuperstructureState.INTAKE)
                        .alongWith(new ConditionalCommand(new RumbleCommand(Seconds.of(1)), Commands.none(),()->enableRumble))
                        .until(endEffectorSubsystem::isFrontEE)
        );
    }
}
