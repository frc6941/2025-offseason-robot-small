package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffectorSubsystem extends SubsystemBase {
    public static final String NAME = "EndEffectorArm";
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();

    private final BeambreakIOInputsAutoLogged frontBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged endBeambreakInputs = new BeambreakIOInputsAutoLogged();

    // Roller motor control
    private final RollerIO rollerIO;
    private final BeambreakIO frontBeamBreakIO;
    private final BeambreakIO endBeamBreakIO;

    @Getter
    @AutoLogOutput(key = "EndEffectorArm/setPoint")
    private double wantedAngle = 0.0;
    @Getter
    @AutoLogOutput(key = "EndEffectorArm/atGoal")
    private boolean atGoal = false;
    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/frontEE")
    private boolean frontEE = false;
    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/endEE")
    private boolean endEE = false;


    public EndEffectorSubsystem(
            RollerIO rollerIO,
            BeambreakIO frontBeamBreakIO,
            BeambreakIO endBeamBreakEndIO) {
        this.rollerIO = rollerIO;
        this.frontBeamBreakIO = frontBeamBreakIO;
        this.endBeamBreakIO = endBeamBreakEndIO;

        // Apply initial PID gains using NTParam values
        rollerIO.updateConfigs(
                EndEffectorParamsNT.ROLLER_KP.getValue(),
                EndEffectorParamsNT.ROLLER_KI.getValue(),
                EndEffectorParamsNT.ROLLER_KD.getValue(),
                EndEffectorParamsNT.ROLLER_KA.getValue(),
                EndEffectorParamsNT.ROLLER_KV.getValue(),
                EndEffectorParamsNT.ROLLER_KS.getValue()
        );
    }

    public void periodic() {
        // Update inputs from hardware
        rollerIO.updateInputs(armRollerIOInputs);
        frontBeamBreakIO.updateInputs(frontBeambreakInputs);
        endBeamBreakIO.updateInputs(endBeambreakInputs);
        if (RobotBase.isReal()) {
            frontEE = frontBeambreakInputs.isBeambreakOn;
            endEE = endBeambreakInputs.isBeambreakOn;
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Front Beambreak", frontBeambreakInputs);
        Logger.processInputs(NAME + "/End Beambreak", endBeambreakInputs);
        Logger.processInputs(NAME + "/Roller", armRollerIOInputs);
        Logger.recordOutput(NAME + "/intakeFinished", intakeFinished());
        Logger.recordOutput(NAME + "/hasCoral", hasCoral());


        // Update tunable numbers if tuning is enabled
        if (Constants.TUNING && EndEffectorParamsNT.isAnyChanged()) {
            // Update roller PID gains if tuning is enabled\[]
            rollerIO.updateConfigs(
                    EndEffectorParamsNT.ROLLER_KP.getValue(),
                    EndEffectorParamsNT.ROLLER_KI.getValue(),
                    EndEffectorParamsNT.ROLLER_KD.getValue(),
                    EndEffectorParamsNT.ROLLER_KA.getValue(),
                    EndEffectorParamsNT.ROLLER_KV.getValue(),
                    EndEffectorParamsNT.ROLLER_KS.getValue()
            );
        }
        LoggedTracer.record("EndEffectorArm");
    }

    public void setRollerVoltage(double voltage) {
        rollerIO.setVoltage(voltage);
    }

    public void hold() {
        rollerIO.setVoltage(EndEffectorParamsNT.CORAL_HOLD_VOLTAGE.getValue());
    }

    public void stopRoller() {
        rollerIO.stop();
    }

    public boolean intakeFinished() {
        return endEE && !frontEE;
    }

    public boolean coralInMiddle() {
        return endEE && frontEE;
    }

    public boolean hasCoral() {
        return endEE || frontEE;
    }
} 