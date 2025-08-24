package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;

public class Right5L3Auto extends AutoRoutine {


    public Right5L3Auto() {
        super("Right5L3Auto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('F', elevatorSetpoint.L3),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('D', elevatorSetpoint.L3),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('C', elevatorSetpoint.L3),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                // DO NOT GO TO E SIDE
                AutoActions.autoScore('B', elevatorSetpoint.L3),
                AutoActions.autoIntake(true),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('A', elevatorSetpoint.L3),
                AutoActions.autoIntake(true)
        );
    }
}
