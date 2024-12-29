
package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

import java.util.function.BooleanSupplier;

public class WaitForCondition implements Command {
    private final BooleanSupplier condition;
    private boolean hasStarted = false;

    public WaitForCondition(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public void start() {
        hasStarted = true;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end() {
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
