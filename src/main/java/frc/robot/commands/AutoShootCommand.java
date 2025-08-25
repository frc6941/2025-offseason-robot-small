package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ElevatorCommonNT;
import frc.robot.Robot;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureState;
import frc.robot.subsystems.SuperstructureStateData;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.swerve.Swerve;

import java.util.function.BooleanSupplier;

public class AutoShootCommand extends SequentialCommandGroup {

    public AutoShootCommand(
            Swerve swerve,
            IndicatorSubsystem indicatorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            EndEffectorSubsystem endEffectorSubsystem,
            Superstructure superstructure,
            ElevatorSetPoint setPoint,
            BooleanSupplier shoot
    ) {
        addCommands(
                Commands.race(
                        Commands.sequence(
                                new ReefAimCommand(swerve, indicatorSubsystem),
                                superstructure.runGoal(Superstructure.elevatorNormal(setPoint))
                        ),
                        Commands.waitUntil(shoot)
                ),
                Commands.waitUntil(elevatorSubsystem::isAtGoal),
                new WaitCommand(0.5).onlyIf(Robot::isSimulation),
                superstructure.runGoal(Superstructure.elevatorforShoot(setPoint))
                        .until(()->!endEffectorSubsystem.hasCoral())
                        .andThen(superstructure.runGoal(()->SuperstructureState.IDLE))
        );
    }

    public AutoShootCommand(
            Swerve swerve,
            IndicatorSubsystem indicatorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            EndEffectorSubsystem endEffectorSubsystem,
            Superstructure superstructure,
            ElevatorSetPoint setPoint,
            char goal
    ) {
        addCommands(
                Commands.sequence(
                        new ReefAimCommand(swerve, indicatorSubsystem, goal),
                        superstructure.runGoal(Superstructure.elevatorNormal(setPoint)).until(elevatorSubsystem::isAtGoal)
                ),
                new WaitCommand(0.5).onlyIf(Robot::isSimulation),
                superstructure.runGoal(Superstructure.elevatorforShoot(setPoint)).until(()->!endEffectorSubsystem.hasCoral()).andThen(superstructure.runGoal(SuperstructureState.IDLE)).until(elevatorSubsystem::isAtGoal)
        );
    }
}
