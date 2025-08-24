package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;

public class Left5L3Auto extends AutoRoutine {


    public Left5L3Auto() {
        super("Left5L3Auto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('I', elevatorSetpoint.L3),
                AutoActions.autoIntake(false),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('K', elevatorSetpoint.L3),
                AutoActions.autoIntake(false),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('L', elevatorSetpoint.L3),
                AutoActions.autoIntake(false),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                // DO NOT GO TO J SIDE
                AutoActions.autoScore('A', elevatorSetpoint.L3),
                AutoActions.autoIntake(false),
                // AutoActions.setLevel(DestinationSupplier.elevatorSetpoint.L3),
                AutoActions.autoScore('B', elevatorSetpoint.L3),
                AutoActions.autoIntake(false)
        );
    }
}
