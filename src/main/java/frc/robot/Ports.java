package frc.robot;

import static frc.robot.Constants.CANIVORE_CAN_BUS_NAME;

public enum Ports {
    END_EFFECTOR(13, CANIVORE_CAN_BUS_NAME),
    ELEVATOR_MAIN(14, CANIVORE_CAN_BUS_NAME),
    ELEVATOR_FOLLOWER(15, CANIVORE_CAN_BUS_NAME);
    
    public final int id;
    public final String bus;

    private Ports(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}