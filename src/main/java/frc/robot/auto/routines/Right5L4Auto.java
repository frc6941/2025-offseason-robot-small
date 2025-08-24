package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;

public class Right5L4Auto extends AutoRoutine {


    public Right5L4Auto() {
        super("RightStationIntakeAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
                AutoActions.autoScore('E',elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('D',elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('C',elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                // DO NOT GO TO E SIDE
                AutoActions.autoScore('B',elevatorSetpoint.L4),
                AutoActions.autoIntake(true),
                AutoActions.autoScore('A',elevatorSetpoint.L4),
                AutoActions.autoIntake(true)
        );
    }
}
