package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;

public class Left3L4And2L3Auto extends AutoRoutine {


    public Left3L4And2L3Auto() {
        super("LeftStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('I'),
                AutoActions.autoIntake(false),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('K'),
                AutoActions.autoIntake(false),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('L'),
                AutoActions.autoIntake(false),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('K'),
                AutoActions.autoIntake(false),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('L'),
                AutoActions.autoIntake(false)
        );
    }
}
