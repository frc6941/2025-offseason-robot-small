package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;

public class Left3L4And2L3Auto extends AutoRoutine {


    public Left3L4And2L3Auto() {
        super("LeftStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                AutoActions.autoScore('I', ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
                AutoActions.autoScore('K', ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
                AutoActions.autoScore('L',ElevatorSetPoint.L4),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('K',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false),
//                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('L',ElevatorSetPoint.L3),
                AutoActions.autoIntake(false)
        );
    }
}
