package frc.robot.subsystems;

import frc.robot.ElevatorCommonNT;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

/**
 * Represents a pose of the superstructure with suppliers for dynamic position control.
 * Uses suppliers to enable both dynamic and static positions.
 */
public record SuperstructurePose(DoubleSupplier elevatorHeight) {

    @Getter
    @RequiredArgsConstructor
    public enum Preset {
        IDLE("IDLE", ElevatorCommonNT.INTAKE_EXTENSION_METERS::getValue),
        L2("L2", ElevatorCommonNT.L2_EXTENSION_METERS::getValue),
        L3("L3", ElevatorCommonNT.L3_EXTENSION_METERS::getValue),
        L4("L4", ElevatorCommonNT.L4_EXTENSION_METERS::getValue),
        P1("P1", ElevatorCommonNT.P1_EXTENSION_METERS::getValue),
        P2("P2", ElevatorCommonNT.P2_EXTENSION_METERS::getValue);


        private final SuperstructurePose pose;


        Preset(String name, DoubleSupplier elevatorHeight) {
            this(
                    new SuperstructurePose(
                            elevatorHeight));
        }



    }
}