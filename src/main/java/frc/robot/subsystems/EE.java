package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EECommonNT;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.EECommonNT.EEGearing.*;

import java.util.function.DoubleSupplier;

public class EE extends SubsystemBase{

    public static final String NAME = "EndEffector";
    private final RollerIOInputsAutoLogged armRollerIOInputs = new RollerIOInputsAutoLogged();

    private final BeambreakIO coralBeambreakIO;

    private final BeambreakIOInputsAutoLogged coralBeambreakInputs = new BeambreakIOInputsAutoLogged();

    private final RollerIO rollerIO;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasCoral")
    private boolean hasCoral = false;

    public EE(
            RollerIO rollerIO,
            BeambreakIO coralBeambreakIO,
            BeambreakIO algaeBeambreakIO) {
        this.rollerIO = rollerIO;
        this.coralBeambreakIO = coralBeambreakIO;

        // Apply initial PID gains
        rollerIO.updateConfigs(
            END_EFFECTOR_ARM_ROLLER_KP.getValue(),
            END_EFFECTOR_ARM_ROLLER_KI.getValue(),
            END_EFFECTOR_ARM_ROLLER_KD.getValue(),
            END_EFFECTOR_ARM_ROLLER_KA.getValue(),
            END_EFFECTOR_ARM_ROLLER_KV.getValue(),
            END_EFFECTOR_ARM_ROLLER_KS.getValue()
        );
    }
    @Override
    public void periodic() {
        coralBeambreakIO.updateInputs(coralBeambreakInputs);
        rollerIO.updateInputs(armRollerIOInputs);
        if (RobotBase.isReal()) {
            // Update gamepiece tracking
            //TODO: add Debouncer or filter to prevent false positives
            hasCoral = coralBeambreakInputs.isBeambreakOn;
            SmartDashboard.putBoolean("GamePiece/EEHasCoral", hasCoral);
        }
        Logger.processInputs(NAME + "/Coral Beambreak", coralBeambreakInputs);

        Logger.processInputs(NAME + "/Roller", armRollerIOInputs);

        if(EECommonNT.isAnyChanged()){
            rollerIO.updateConfigs(
                END_EFFECTOR_ARM_ROLLER_KP.getValue(),
                END_EFFECTOR_ARM_ROLLER_KI.getValue(),
                END_EFFECTOR_ARM_ROLLER_KD.getValue(),
                END_EFFECTOR_ARM_ROLLER_KA.getValue(),
                END_EFFECTOR_ARM_ROLLER_KV.getValue(),
                END_EFFECTOR_ARM_ROLLER_KS.getValue()
            );
        }
        LoggedTracer.record("EndEffectorArm");
    }

    public void setRollerVoltage(DoubleSupplier voltage) {
        rollerIO.setVoltage(voltage.getAsDouble());
    }

    public void stopRoller() {
        rollerIO.stop();
    }
}
