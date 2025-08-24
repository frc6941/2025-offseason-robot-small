package lib.ironpulse.command;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/*
 * This class is edited from ConditionalCommand
 * Difference: The requirements will be load in initialize() instead of the DriverConditionalCommand()
 */
public class DriverConditionalCommand extends Command {
    private final Command m_onTrue;
    private final Command m_onFalse;
    private final BooleanSupplier m_condition;
    private Command m_selectedCommand;

    public DriverConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        this.m_onTrue = (Command) ErrorMessages.requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
        this.m_onFalse = (Command) ErrorMessages.requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");
        this.m_condition = (BooleanSupplier) ErrorMessages.requireNonNullParam(condition, "condition", "ConditionalCommand");
        CommandScheduler.getInstance().registerComposedCommands(new Command[]{onTrue, onFalse});
    }

    @Override
    public void initialize() {
        if (this.m_condition.getAsBoolean()) {
            this.m_selectedCommand = this.m_onTrue;
        } else {
            this.m_selectedCommand = this.m_onFalse;
        }

        this.addRequirements(this.m_selectedCommand.getRequirements());
        this.m_selectedCommand.initialize();
    }

    @Override
    public void execute() {
        this.m_selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        this.m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.m_selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return this.m_onTrue.runsWhenDisabled() && this.m_onFalse.runsWhenDisabled();
    }

    @Override
    public Command.InterruptionBehavior getInterruptionBehavior() {
        return this.m_onTrue.getInterruptionBehavior() != InterruptionBehavior.kCancelSelf && this.m_onFalse.getInterruptionBehavior() != InterruptionBehavior.kCancelSelf ? InterruptionBehavior.kCancelIncoming : InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        Command var10002 = this.m_onTrue;
        Objects.requireNonNull(var10002);
        builder.addStringProperty("onTrue", var10002::getName, (Consumer) null);
        var10002 = this.m_onFalse;
        Objects.requireNonNull(var10002);
        builder.addStringProperty("onFalse", var10002::getName, (Consumer) null);
        builder.addStringProperty("selected", () -> this.m_selectedCommand == null ? "null" : this.m_selectedCommand.getName(), (Consumer) null);
    }
}