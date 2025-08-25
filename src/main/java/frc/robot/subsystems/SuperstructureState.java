package frc.robot.subsystems;

import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.SuperstructurePose.Preset;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
    START(createState(Preset.IDLE,EndEffectorParamsNT.IDLE_VOLTAGE::getValue)),
    IDLE(createState(Preset.IDLE, EndEffectorParamsNT.IDLE_VOLTAGE::getValue)),
    INTAKE(createState(Preset.IDLE, EndEffectorParamsNT.CORAL_INTAKE_VOLTAGE::getValue)),
    INDEX(createState(Preset.IDLE, EndEffectorParamsNT.CORAL_INDEX_VOLTAGE::getValue)),
    OUTTAKE(createState(Preset.IDLE, EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)),
    L2(createState(Preset.L2, EndEffectorParamsNT.IDLE_VOLTAGE::getValue)),
    L2_EJECT(createState(Preset.L2, EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)),
    L3(createState(Preset.L3, EndEffectorParamsNT.IDLE_VOLTAGE::getValue)),
    L3_EJECT(createState(Preset.L3, EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)),
    L4(createState(Preset.L4, EndEffectorParamsNT.IDLE_VOLTAGE::getValue)),
    L4_EJECT(createState(Preset.L4, EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)),
    P1(createState(Preset.P1, EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue)),
    P2(createState(Preset.P2, EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue));


    private final SuperstructureStateData value;

    // Creates a basic state with just the pose
    private static SuperstructureStateData createState(Preset preset, DoubleSupplier eeVoltage) {
        return SuperstructureStateData.builder()
                .pose(preset.getPose())
                .endEffectorVolts(eeVoltage)
                .build();
    }


}