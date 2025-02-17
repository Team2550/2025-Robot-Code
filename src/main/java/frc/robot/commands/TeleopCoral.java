package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.CoralHandlerSubsystem.CoralHandlerStateMachine.State;

public class TeleopCoral extends Command {
    private CoralHandlerSubsystem s_CoralHandler;
    private DoubleSupplier operatorActionAxisSup;
    private BooleanSupplier l2ButtonPressed;
    private BooleanSupplier l3ButtonPressed;
    private BooleanSupplier l4ButtonPressed;
    private IntSupplier operatorDPadPOV;
    private State currentState;
    private State newState;

    public TeleopCoral(CoralHandlerSubsystem coralHandlerSubsystem, DoubleSupplier operatorActionAxisSup, BooleanSupplier l2ButtonPressed, BooleanSupplier l3ButtonPressed, BooleanSupplier l4ButtonPressed, IntSupplier operatorDPadPOV) {
        this.s_CoralHandler = coralHandlerSubsystem;
        addRequirements(s_CoralHandler);
        this.operatorActionAxisSup = operatorActionAxisSup;
        this.l2ButtonPressed = l2ButtonPressed;
        this.l3ButtonPressed = l3ButtonPressed;
        this.l4ButtonPressed = l4ButtonPressed;
        this.operatorDPadPOV = operatorDPadPOV;
    }

    @Override
    public void execute() {

        if (l2ButtonPressed.getAsBoolean()) {
            newState = State.L2;
        } else if (l3ButtonPressed.getAsBoolean()) {
            newState = State.L3;
        } else if (l4ButtonPressed.getAsBoolean()) {
            newState = State.L4;
        }

        if (operatorActionAxisSup.getAsDouble() > 0.7) {
            s_CoralHandler.runStatePath(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(currentState, newState))
                .andThen(new InstantCommand(() -> {
                    currentState = newState;
                })).schedule();
        }
    }
}
