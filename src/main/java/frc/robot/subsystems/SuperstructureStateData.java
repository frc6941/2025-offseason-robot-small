package frc.robot.subsystems;

import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;

import java.util.function.DoubleSupplier;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
    @Builder.Default
    private final SuperstructurePose pose = new SuperstructurePose(() -> 0.0);

    @Builder.Default private final DoubleSupplier endEffectorVolts = () -> 0.0;
} 