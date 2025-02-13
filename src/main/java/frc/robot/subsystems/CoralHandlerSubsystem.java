package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.CoralHandlerConstants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;
    

    private static final double multiplier = 7.07;

    
    //TODO: DO AWAY WITH OLD STATE MACHINE AND USE "THE MONSTROSITY"
    class CoralHandlerStateMachine {
        enum State
        {
            Rest(-1,-1, false),
            CoralPickup1(-1,-1, false),
            CoralPickup2(-1,-1, true),
            L1(-1,-1, false),
            L2(-1,-1, false),
            L3(-1,-1, true),
            L4(-1,-1, true);

            private double elevatorHeightMeters;
            private double armAngleDegrees;
            private boolean isHopperIn;

            State(double elevatorHeightMeters, double armAngleDegrees, boolean isHopperIn) {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.armAngleDegrees = armAngleDegrees;
                this.isHopperIn = isHopperIn;
            }

            double getElevatorHeightMeters() {
                return this.elevatorHeightMeters;
            }

            double getArmAngleDegrees() {
                return this.armAngleDegrees;
            }

        }
        // "THE MONSTROSITY"
        enum StateTransitionPath {
            RestToL1(State.Rest, State.L1),
            RestToL2(State.Rest, State.L2),
            RestToL3(State.Rest, State.L3),
            RestToL4(State.Rest, State.L4),
            RestToCoralPickup1(State.Rest, State.CoralPickup1),
            RestToCoralPickup2(State.Rest, State.CoralPickup2, State.CoralPickup1),

            L1ToRest(State.L1, State.Rest),
            L1ToL2(State.L1, State.L2),
            L1ToL3(State.L1, State.L3, State.Rest),
            L1ToL4(State.L1, State.L4, State.Rest),
            L1ToCoralPickup1(State.L1, State.CoralPickup1, State.Rest),
            L1ToCoralPickup2(State.L1, State.CoralPickup2, State.Rest, State.CoralPickup1),

            L2ToRest(State.L2, State.Rest),
            L2ToL1(State.L2, State.L1),
            L2ToL3(State.L2, State.L3, State.Rest),
            L2ToL4(State.L2, State.L4, State.Rest),
            L2ToCoralPickup1(State.L2, State.CoralPickup1, State.Rest),
            L2ToCoralPickup2(State.L2, State.CoralPickup2, State.Rest, State.CoralPickup1),

            L3ToRest(State.L3, State.Rest),
            L3ToL1(State.L3, State.L1, State.Rest),
            L3ToL2(State.L3, State.L2, State.Rest),
            L3ToL4(State.L3, State.L4),
            L3ToCoralPickup1(State.L3, State.CoralPickup1, State.Rest),
            L3ToCoralPickup2(State.L3, State.CoralPickup2, State.Rest, State.CoralPickup1),

            L4ToRest(State.L4, State.Rest),
            L4ToL1(State.L4, State.L1, State.Rest),
            L4ToL2(State.L4, State.L2, State.Rest),
            L4ToL3(State.L4, State.L3),
            L4ToCoralPickup1(State.L4, State.CoralPickup1, State.Rest),
            L4ToCoralPickup2(State.L4, State.CoralPickup2, State.Rest, State.CoralPickup1),

            CoralPickup1ToRest(State.CoralPickup1, State.Rest),
            CoralPickup1ToL1(State.CoralPickup1, State.L1, State.Rest),
            CoralPickup1ToL2(State.CoralPickup1, State.L2, State.Rest),
            CoralPickup1ToL3(State.CoralPickup1, State.L3, State.Rest),
            CoralPickup1ToL4(State.CoralPickup1, State.L4, State.Rest),
            CoralPickup1ToCoralPickup2(State.CoralPickup1, State.CoralPickup2),

            CoralPickup2ToRest(State.CoralPickup2, State.Rest, State.CoralPickup1),
            CoralPickup2ToL1(State.CoralPickup2, State.L1, State.CoralPickup1, State.Rest),
            CoralPickup2ToL2(State.CoralPickup2, State.L2, State.CoralPickup1, State.Rest),
            CoralPickup2ToL3(State.CoralPickup2, State.L3, State.CoralPickup1, State.Rest),
            CoralPickup2ToL4(State.CoralPickup2, State.L4, State.CoralPickup1, State.Rest),
            CoralPickup2ToCoralPickup1(State.CoralPickup2, State.CoralPickup1);

            StateTransitionPath(State initial, State endpoint, State... intermediary) {

            }
        }

        State currentState = State.Rest;
        Map<State, State[]> allowedStateTransitions = new HashMap<State, State[]>() {{
            put(State.Rest, new State[]{State.L1, State.L2, State.L3, State.L4, State.CoralPickup1});
            put(State.CoralPickup1, new State[]{State.Rest, State.CoralPickup2});
            put(State.CoralPickup2, new State[]{State.CoralPickup1});
            put(State.L1, new State[]{State.L2, State.Rest});
            put(State.L2, new State[]{State.L1, State.Rest});
            put(State.L3, new State[]{State.L4, State.Rest});
            put(State.L4, new State[]{State.L3, State.Rest});
        }};

        CoralHandlerStateMachine() {}

        boolean attemptStateTransition(State newState) {
            if (Arrays.asList(allowedStateTransitions.get(currentState)).contains(newState)) {
                currentState = newState;
                return true;
            }
            return false;
        }
    }

    private CoralHandlerStateMachine coralHandlerStateMachine;

    public CoralHandlerSubsystem() {

        //Set Up Elevator Motor
        mElevatorMotor = new TalonFX(9);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorFXConfig);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorPIDConfig);
        
        mElevatorMotor.setPosition(0);
        mElevatorRequest = new PositionVoltage(0).withSlot(0);

        //Set Up Arm Motor
        mArmMotor = new TalonFX(10);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorFXConfig);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorPIDConfig);
        
        mArmMotor.setPosition(-0.25);
        //Right is 0
        //Up is 90
        //Left is 180
        //Down is 270
        mArmRequest = new PositionVoltage(90).withSlot(0);

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
    }

    public Command intake() {
        return null;
    }

    public Command expel() {
        Runnable selectedHeightCommand = () -> {};
        int scoringPosition = (int)mSelectedScoringHeightEntry.getInteger(-1);
        


        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return this.run(selectedHeightCommand);
    }

    public Command zero() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public void changeToIntakePose() {}

    // Sets the height of the elevator from the bottom in meters
    public Command moveElevatorTo(double height) {
        if (0 <= height && height <= 0.762) {
            return this.run(() -> {
                mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(height, 0.13) * multiplier));
            });
        } else {
            return null;
        }
    }

    // Returns the elevator to the bottom at its predefined resting position
    public Command rest() {
        return this.run(() -> {
            mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(CoralHandlerConstants.restPose, 0.13) * multiplier));
        });
    }

    // Stops the elevator motor at its current position
    public void stopElevatorMotor() {
        mElevatorMotor.set(0);
    }

    // Stops the arm at its current position
    public void stopArmMotor(){
        mArmMotor.set(0);
    }

    public Command changeArmPose(Rotation2d pose) {
        // mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees() * 62.5) / 360));
        if (-180 <= pose.getDegrees() && pose.getDegrees() <= 180) {
            return this.run(() -> {
                mArmMotor.setControl(mArmRequest.withPosition((pose.getDegrees() * 62.5) / 360));
            });
        } else {
            return null;
        }
    }
}
