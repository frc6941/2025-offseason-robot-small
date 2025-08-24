package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ElevatorCommonNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure;
import frc.robot.subsystems.RobotSuperStructure.*;
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
            RobotSuperStructure robotSuperStructure,
            elevatorSetpoint setPoint,
            BooleanSupplier shoot
    ) {
        addCommands(
                Commands.race(
                        Commands.sequence(
                                new ReefAimCommand(swerve, indicatorSubsystem),
                                robotSuperStructure.runGoal(()->RobotSuperStructure.elevatorNormalSetpointMapping(setPoint))
                        ),
                        Commands.waitUntil(shoot)
                ),
                Commands.waitUntil(elevatorSubsystem::isAtGoal),
                robotSuperStructure.runGoal(()->RobotSuperStructure.elevatorScoringSetpointMapping(setPoint)).andThen(robotSuperStructure.runGoal(RobotSuperstructuresState.IDLE))
        );
    }

    public AutoShootCommand(
            Swerve swerve,
            IndicatorSubsystem indicatorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            EndEffectorSubsystem endEffectorSubsystem,
            RobotSuperStructure robotSuperStructure,
            elevatorSetpoint setPoint,
            char goal
    ) {
        addCommands(
                Commands.sequence(
                        new ReefAimCommand(swerve, indicatorSubsystem, goal),
                        robotSuperStructure.runGoal(()->RobotSuperStructure.elevatorNormalSetpointMapping(setPoint))
                ),
                Commands.waitUntil(elevatorSubsystem::isAtGoal),
                robotSuperStructure.runGoal(()->RobotSuperStructure.elevatorScoringSetpointMapping(setPoint)).andThen(robotSuperStructure.runGoal(RobotSuperstructuresState.IDLE))
        );
    }
}
