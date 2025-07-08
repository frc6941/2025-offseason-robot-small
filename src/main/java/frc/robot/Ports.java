package frc.robot;

import static frc.robot.Constants.CANIVORE_CAN_BUS_NAME;
import static frc.robot.Constants.RIO_CAN_BUS_NAME;

public enum Ports {
	// ALGAE_DEPLOY(8, CANIVORE_CAN_BUS_NAME),
	// ALGAE_ROLLERS(9, RIO_CAN_BUS_NAME),
	// CORAL_DEPLOY(10, CANIVORE_CAN_BUS_NAME),
	// CORAL_ROLLERS(11, RIO_CAN_BUS_NAME),
	// CORAL_INDEXER(12, CANIVORE_CAN_BUS_NAME),
	END_EFFECTOR(13, CANIVORE_CAN_BUS_NAME),
	ELEVATOR_MAIN(14, CANIVORE_CAN_BUS_NAME),
	ELEVATOR_FOLLOWER(15, CANIVORE_CAN_BUS_NAME),
	// PIVOT(16, CANIVORE_CAN_BUS_NAME),
	// CLIMBER(17, CANIVORE_CAN_BUS_NAME),
	// CLIMBER_ROLLERS(18, CANIVORE_CAN_BUS_NAME),
	// CANDLE(21, CANIVORE_CAN_BUS_NAME),

	// // END_EFFECTOR_CORAL_BREAMBREAK(RobotConstants.isComp ? 1 : 8, "RioDigitalIn"),
	// // END_EFFECTOR_ALGAE_BEAMBREAK(RobotConstants.isComp ? 0 : 7, "RioDigitalIn"),
	// // INDEXER_BEAMBREAK(RobotConstants.isComp ? 8 : 6, "RioDigitalIn"),

	// ENCODER_41T(4, CANIVORE_CAN_BUS_NAME),
	// ENCODER_39T(5, CANIVORE_CAN_BUS_NAME),

	PHYSICAL_BUTTON(9, "RioDigitalIn");

	public final int id;
	public final String bus;

	private Ports(int id, String bus) {
		this.id = id;
		this.bus = bus;
	}
}