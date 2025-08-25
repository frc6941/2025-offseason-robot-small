package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;

public class Left5L3Auto extends AutoRoutine {


    public Left5L3Auto() {
        super("Left5L3Auto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('I', ElevatorSetPoint.L3),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('K',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('L',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                // DO NOT GO TO J SIDE
                AutoActions.autoScore('A',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('B',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false)
        );
    }
}
