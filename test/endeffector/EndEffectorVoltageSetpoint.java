package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.Units;
import lib.frcteam1678.io.MotorIO.Setpoint;

/**
 * 枚举表示所有可能的端效应器设定点。
 */
public enum EndEffectorVoltageSetpoint {
    // 基本设定点
    IDLE(Setpoint.withCoastSetpoint()),
    SPIT(Setpoint.withVoltageSetpoint(EndEffectorConstants.kSpitVoltage)),

    // 珊瑚相关设定点
    CORAL_FEED(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralIntakeVoltage)),
    CORAL_HOLD(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralHoldVoltage)),
    CORAL_SCORE_L1(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL1)),
    CORAL_SCORE_L2(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL2)),
    CORAL_SCORE_L3(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL3)),
    CORAL_SCORE_L4(Setpoint.withVoltageSetpoint(EndEffectorConstants.kCoralOuttakeVoltageL4)),
    SOFT_CORAL_SCORE(Setpoint.withVoltageSetpoint(EndEffectorConstants.kSoftCoralOuttakeVoltage)),

    // 站点摄取设定点
    STATION_INTAKE(Setpoint.withVoltageSetpoint(EndEffectorConstants.kStationIntakeVoltage));

    private final Setpoint setpoint;

    EndEffectorVoltageSetpoint(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * 获取与此枚举值关联的Setpoint对象。
     * @return Setpoint对象
     */
    public Setpoint getSetpoint() {
        return setpoint;
    }
}