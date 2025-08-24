package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;

public class Right3L4And2L3Auto extends AutoRoutine {


    public Right3L4And2L3Auto() {
        super("RightStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('F', elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('D', elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L4),
                AutoActions.autoScore('C', elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('D', elevatorSetpoint.L3),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('C', elevatorSetpoint.L3),
                AutoActions.autoIntake(true)
        );
    }
}
