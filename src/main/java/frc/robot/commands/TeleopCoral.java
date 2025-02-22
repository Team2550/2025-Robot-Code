package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHandlerSubsystem;

public class TeleopCoral extends Command {
    private CoralHandlerSubsystem s_CoralHandler;
    private DoubleSupplier operatorActionAxisSup;

    public TeleopCoral(CoralHandlerSubsystem coralHandlerSubsystem, DoubleSupplier operatorActionAxisSup) {
        this.s_CoralHandler = coralHandlerSubsystem;
        addRequirements(s_CoralHandler);
        this.operatorActionAxisSup = operatorActionAxisSup;
    }

    @Override
    public void execute() {

        // if (l2ButtonPressed.getAsBoolean()) {
        //     newState = State.L2;
        // } else if (l3ButtonPressed.getAsBoolean()) {
        //     newState = State.L3;
        // } else if (l4ButtonPressed.getAsBoolean()) {
        //     newState = State.L4;
        // }

        if (operatorActionAxisSup.getAsDouble() > 0.7) {
        }
    }
}
