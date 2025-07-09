package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import lib.frcteam1678.io.MotorIO.Setpoint;

/**
 * Enum representing all possible elevator setpoints.
 */
public enum ElevatorSetpoint {
    // Voltage-based setpoints
    JOG_UP(Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(0.5, Units.Volts))),
    JOG_DOWN(Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(-0.5, Units.Volts))),
    HOLD_UP(Setpoint.withVoltageSetpoint(Voltage.ofBaseUnits(0, Units.Volts))),
    
    // Position-based setpoints
    L4_SCORE(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL4ScoringHeight))),
    L3_SCORE(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL3ScoringHeight))),
    L2_SCORE(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL2ScoringHeight))),
    L1_SCORE(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kL1ScoringHeight))),
    CLEAR_LOW_HEIGHT(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kClearLowPosition))),
    CLEAR_HIGH_HEIGHT(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kClearHighPosition))),
    CORAL_HOLD(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kCoralHoldPosition))),
    STOW(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition))),
    EE_UNJAM(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kEEUnjameHeight))),
    CORAL_STATION(Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(ElevatorConstants.kStationIntake)));

    private final Setpoint setpoint;

    ElevatorSetpoint(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Gets the Setpoint object associated with this enum value.
     * @return The Setpoint object
     */
    public Setpoint getSetpoint() {
        return setpoint;
    }
}