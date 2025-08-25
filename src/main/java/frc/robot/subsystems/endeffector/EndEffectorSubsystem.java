package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class EndEffectorSubsystem extends SubsystemBase {
    public static final String NAME = "EndEffector";
    private final RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    private final BeambreakIOInputsAutoLogged frontBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged endBeambreakInputs = new BeambreakIOInputsAutoLogged();

    // Roller motor control
    private final RollerIO rollerIO;
    private final BeambreakIO frontBeamBreakIO;
    private final BeambreakIO endBeamBreakIO;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffector/frontEE")
    private boolean frontEE = false;
    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffector/endEE")
    private boolean endEE = false;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasCoral")
    private boolean hasCoral = false;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/intakeFinished")
    private boolean intakeFinished = false;


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
        rollerIO.updateInputs(rollerIOInputs);
        frontBeamBreakIO.updateInputs(frontBeambreakInputs);
        endBeamBreakIO.updateInputs(endBeambreakInputs);
        if (RobotBase.isReal()) {
            frontEE = frontBeambreakInputs.isBeambreakOn;
            endEE = endBeambreakInputs.isBeambreakOn;
            intakeFinished = endEE && !frontEE;
            hasCoral = endEE || frontEE;
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Front Beambreak", frontBeambreakInputs);
        Logger.processInputs(NAME + "/End Beambreak", endBeambreakInputs);
        Logger.processInputs(NAME + "/Roller", rollerIOInputs);
        Logger.recordOutput(NAME + "/intakeFinished", intakeFinished());
        Logger.recordOutput(NAME + "/hasCoral", hasCoral());
        SmartDashboard.putBoolean("EndEffector/hasCoral", hasCoral());
        SmartDashboard.putBoolean("EndEffector/endEE", endEE);
        SmartDashboard.putBoolean("EndEffector/frontEE", frontEE);


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
        LoggedTracer.record("EndEffector");
    }

    public void setRollerVoltage(double voltage) {
        rollerIO.setVoltage(voltage);
    }

    public void setRollerVoltage(DoubleSupplier voltage) {
        rollerIO.setVoltage(voltage.getAsDouble());
    }

    public void hold() {
        rollerIO.setVoltage(EndEffectorParamsNT.CORAL_HOLD_VOLTAGE.getValue());
    }

    public void stopRoller() {
        rollerIO.stop();
    }

    public boolean intakeFinished() {
        return intakeFinished;
    }

    public boolean coralInMiddle() {
        return endEE && frontEE;
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public Command setRolelrVoltage(DoubleSupplier voltage){
        return Commands.runOnce(()->setRollerVoltage(voltage));
    }
} 