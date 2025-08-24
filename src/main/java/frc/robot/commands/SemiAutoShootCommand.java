package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotStateRecorder;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.swerve.Swerve;

public class SemiAutoShootCommand extends ParallelCommandGroup {

    public SemiAutoShootCommand(
            Swerve swerve,
            IndicatorSubsystem indicatorSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            RobotSuperStructure robotSuperStructure,
            elevatorSetpoint setpoint
    ) {
        addCommands(
                new ReefAimCommand(swerve, indicatorSubsystem),
                Commands.sequence(
                        Commands.waitUntil(
                                () -> DestinationSupplier.isSafeToRaise(
                                        RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                                        DestinationSupplier.getInstance().isCoralRight())).onlyIf(
                                () -> setpoint == elevatorSetpoint.L4),
                        robotSuperStructure.runGoal(RobotSuperStructure.elevatorNormalSetpointMapping(setpoint))
                )

        );
    }
}
