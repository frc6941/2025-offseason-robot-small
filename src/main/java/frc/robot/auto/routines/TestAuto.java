package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoRoutine;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.RobotSuperStructure.elevatorSetpoint;

public class TestAuto extends AutoRoutine {
    public TestAuto() {
        super("TestAuto");
    }

    @Override
    public Command getAutoCommand() {
        return Commands.sequence(
            AutoActions.autoScore('C', elevatorSetpoint.L4));
    }
}
