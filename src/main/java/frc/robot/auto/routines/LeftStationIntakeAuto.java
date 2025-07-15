package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;

public class LeftStationIntakeAuto extends AutoRoutine {


    public LeftStationIntakeAuto() {
        super("LeftStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                AutoActions.reverseEE(),
                AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('I'),
                Commands.parallel(
                        AutoActions.toStation(false),
                        AutoActions.intake()
                ),
                AutoActions.autoScore('K'),
                Commands.parallel(
                        AutoActions.toStation(false),
                        AutoActions.intake()
                ),
                AutoActions.autoScore('L'),
                Commands.parallel(
                        AutoActions.toStation(false),
                        AutoActions.intake()
                ),
                AutoActions.autoScore('J'),
                Commands.parallel(
                        AutoActions.toStation(false),
                        AutoActions.intake()
                ), AutoActions.autoScore('A'),
                Commands.parallel(
                        AutoActions.toStation(false),
                        AutoActions.intake()
                )
        );
    }
}
