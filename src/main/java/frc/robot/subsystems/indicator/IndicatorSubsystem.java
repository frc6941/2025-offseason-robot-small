package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.DestinationSupplier;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;

public class IndicatorSubsystem extends SubsystemBase {
    private final IndicatorIO io;
    private final IndicatorIOInputsAutoLogged inputs = new IndicatorIOInputsAutoLogged();
    private final Timer timer = new Timer();
    private IndicatorIO.Patterns currentPattern = IndicatorIO.Patterns.AUTO;
    @Getter
    private IndicatorIO.Patterns lastPattern = IndicatorIO.Patterns.AUTO;

    public IndicatorSubsystem(IndicatorIO io) {
        this.io = io;
    }

    public void setPattern(IndicatorIO.Patterns pattern) {
        if (pattern == currentPattern) {
            io.setPattern(currentPattern);
            return;
        }
        lastPattern = currentPattern;
        currentPattern = pattern;
        io.setPattern(pattern);
        switch (pattern) {
            case AFTER_INTAKE, RESET_ODOM, AIMED, SHOOT -> timer.restart();
            default -> {
            }
        }
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case AFTER_INTAKE, RESET_ODOM, AIMED, SHOOT -> resetLed();
            case AUTO, MANUAL -> setNormal();
            default -> {
            }
        }
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.processInputs("Indicator", inputs);
        LoggedTracer.record("Indicator");
    }

    private void resetLed() {
        if (!timer.hasElapsed(2)) return;
        setNormal();
    }

    public void reset() {
        this.io.reset();
    }

    public void resetToLastPattern() {
        setPattern(lastPattern);
    }

    public void setNormal() {
        if (DestinationSupplier.getInstance().isAuto()) setPattern(IndicatorIO.Patterns.AUTO);
        else setPattern(IndicatorIO.Patterns.MANUAL);
    }
}
