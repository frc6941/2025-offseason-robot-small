package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.EndEffectorParamsNT;
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

    private final BeambreakIOInputsAutoLogged coralBeambreakInputs = new BeambreakIOInputsAutoLogged();
    private final BeambreakIOInputsAutoLogged algaeBeambreakInputs = new BeambreakIOInputsAutoLogged();

    // Roller motor control
    private final RollerIO rollerIO;

    @Getter
    @AutoLogOutput(key = "EndEffectorArm/setPoint")
    private double wantedAngle = 0.0;
    @Getter
    @AutoLogOutput(key = "EndEffectorArm/atGoal")
    private boolean atGoal = false;
    @AutoLogOutput(key = "EndEffectorArm/stopDueToLimit")
    private boolean stopDueToLimit = false;

    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasCoral")
    private boolean hasCoral = false;
    @Getter
    @Setter
    @AutoLogOutput(key = "EndEffectorArm/hasAlgae")
    private boolean hasAlgae = false;

    public EndEffectorSubsystem(
            RollerIO rollerIO) {
        this.rollerIO = rollerIO;

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
        if (RobotBase.isReal()) {
            // Update gamepiece tracking
            //TODO: add Debouncer or filter to prevent false positives
            hasCoral = coralBeambreakInputs.isBeambreakOn;
            hasAlgae = algaeBeambreakInputs.isBeambreakOn;
            SmartDashboard.putBoolean("GamePiece/EEHasCoral", hasCoral);
            SmartDashboard.putBoolean("GamePiece/EEHasAlgae", hasAlgae);
        }

        // Process and log inputs
        Logger.processInputs(NAME + "/Coral Beambreak", coralBeambreakInputs);
        Logger.processInputs(NAME + "/Algae Beambreak", algaeBeambreakInputs);
        Logger.processInputs(NAME + "/Roller", armRollerIOInputs);

        // Update tunable numbers if tuning is enabled
        if (Constants.TUNING && EndEffectorParamsNT.isAnyChanged()) {
            // Update roller PID gains if tuning is enabled
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

    public void stopRoller() {
        rollerIO.stop();
    }
} 