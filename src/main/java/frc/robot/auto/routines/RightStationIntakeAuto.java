package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;

public class RightStationIntakeAuto extends AutoRoutine {


    public RightStationIntakeAuto() {
        super("RightStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                AutoActions.reverseEE(),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('F'),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('D'),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('C'),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('E'),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('B'),
                AutoActions.autoIntake(true)
        );
    }
}
