package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ElevatorCommonNT;
import frc.robot.drivers.DestinationSupplier;
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
            BooleanSupplier shoot
    ) {
        addCommands(
                Commands.race(
                        Commands.parallel(
                                new ReefAimCommand(swerve, indicatorSubsystem),
                                Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true)))
                        ),
                        Commands.waitUntil(shoot)
                ),
                //new ShootCommand(endEffectorSubsystem, indicatorSubsystem),
                Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue()))
        );
    }
}
