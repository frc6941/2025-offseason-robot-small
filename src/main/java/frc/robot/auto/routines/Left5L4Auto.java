package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;

public class Left5L4Auto extends AutoRoutine {


    public Left5L4Auto() {
        super("LeftStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('J', ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('K',ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('L',ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                // DO NOT GO TO J SIDE
                AutoActions.autoScore('A',ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('B',ElevatorSetPoint.L4),
                AutoActions.autoIntake(false)
        );
    }
}
