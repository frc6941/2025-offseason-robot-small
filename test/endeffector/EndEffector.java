package frc.robot.subsystems.endeffector;

import lib.frcteam1678.bases.MotorSubsystem;
import lib.frcteam1678.io.MotorIO.Setpoint;
import lib.frcteam1678.io.MotorIOTalonFX;

public class EndEffector extends MotorSubsystem<MotorIOTalonFX> {
	// 移除所有静态Setpoint常量

	public static final EndEffector m_Instance = new EndEffector();

	private EndEffector() {
		super(EndEffectorConstants.getMotorIO(), "End Effector");
	}

	/**
	 * 应用来自EndEffectorSetpoint枚举的设定点
	 * @param setpoint 要应用的设定点
	 */
	public void applyEndEffectorSetpoint(EndEffectorVoltageSetpoint setpoint) {
		applySetpoint(setpoint.getSetpoint());
	}

	public static Setpoint getCoralScoreSetpoint(lib.ironpulse.utils.Level level) {
		return switch (level) {
			case L4 -> EndEffectorVoltageSetpoint.CORAL_SCORE_L4.getSetpoint();
			case L3 -> EndEffectorVoltageSetpoint.CORAL_SCORE_L3.getSetpoint();
			case L2 -> EndEffectorVoltageSetpoint.CORAL_SCORE_L2.getSetpoint();
			case L1 -> EndEffectorVoltageSetpoint.CORAL_SCORE_L1.getSetpoint();
			default -> EndEffectorVoltageSetpoint.CORAL_SCORE_L4.getSetpoint();
		};
	}
}
