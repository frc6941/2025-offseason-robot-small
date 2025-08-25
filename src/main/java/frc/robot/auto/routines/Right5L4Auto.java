package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;

public class Right5L4Auto extends AutoRoutine {


    public Right5L4Auto() {
        super("RightStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('E', ElevatorSetPoint.L4),
                AutoActions.autoIntake(true),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('D',ElevatorSetPoint.L4),
                AutoActions.autoIntake(true),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('C',ElevatorSetPoint.L4),
                AutoActions.autoIntake(true),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                // DO NOT GO TO E SIDE
                AutoActions.autoScore('B',ElevatorSetPoint.L4),
                AutoActions.autoIntake(true),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('A',ElevatorSetPoint.L4),
                AutoActions.autoIntake(true)
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4)
        );
    }
}
