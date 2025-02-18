package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.CoralHandlerConstants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;
    private DoubleSolenoid mHopperSolenoid;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;
    

    private static final double multiplier = 7.07;

    
    //TODO: DO AWAY WITH OLD STATE MACHINE AND USE "THE MONSTROSITY"
    public class CoralHandlerStateMachine {
        public enum State
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
        public enum StateTransitionPath {
            RestToL1(State.Rest, State.L1),
            RestToL2(State.Rest, State.L2),
            RestToL3(State.Rest, State.L3),
            RestToL4(State.Rest, State.L4),
            RestToCoralPickup1(State.Rest, State.CoralPickup1),
            RestToCoralPickup2(State.Rest, State.CoralPickup1, State.CoralPickup2),
    
            L1ToRest(State.L1, State.Rest),
            L2ToRest(State.L2, State.Rest),
            L3ToRest(State.L3, State.Rest),
            L4ToRest(State.L4, State.Rest),
    
            L1ToL2(State.L1, State.L2),
            L2ToL1(State.L2, State.L1),
            L3ToL4(State.L3, State.L4),
            L4ToL3(State.L4, State.L3),
    
            L1ToL3(State.L1, State.Rest, State.L3),
            L1ToL4(State.L1, State.Rest, State.L4),
            L2ToL3(State.L2, State.Rest, State.L3),
            L2ToL4(State.L2, State.Rest, State.L4),
    
            L3ToL1(State.L3, State.Rest, State.L1),
            L3ToL2(State.L3, State.Rest, State.L2),
            L4ToL1(State.L4, State.Rest, State.L1),
            L4ToL2(State.L4, State.Rest, State.L2),
    
            CoralPickup1ToRest(State.CoralPickup1, State.Rest),
            CoralPickup2ToRest(State.CoralPickup2, State.Rest, State.CoralPickup1),
    
            CoralPickup1ToCoralPickup2(State.CoralPickup1, State.CoralPickup2),
            CoralPickup2ToCoralPickup1(State.CoralPickup2, State.CoralPickup1),
    
            CoralPickup1ToL1(State.CoralPickup1, State.Rest, State.L1),
            CoralPickup1ToL2(State.CoralPickup1, State.Rest, State.L2),
            CoralPickup1ToL3(State.CoralPickup1, State.Rest, State.L3),
            CoralPickup1ToL4(State.CoralPickup1, State.Rest, State.L4),
    
            CoralPickup2ToL1(State.CoralPickup2, State.CoralPickup1, State.Rest, State.L1),
            CoralPickup2ToL2(State.CoralPickup2, State.CoralPickup1, State.Rest, State.L2),
            CoralPickup2ToL3(State.CoralPickup2, State.CoralPickup1, State.Rest, State.L3),
            CoralPickup2ToL4(State.CoralPickup2, State.CoralPickup1, State.Rest, State.L4);
    
            private final List<State> path;
    
            StateTransitionPath(State... states) {
                this.path = Arrays.asList(states);
            }
    
            public List<State> getPath() {
                return path;
            }
    
            public State getStart() {
                return path.get(0);
            }
    
            public State getEnd() {
                return path.get(path.size() - 1);
            }
    
            public List<State> getIntermediaryStates() {
                return path.size() > 2 ? path.subList(1, path.size() - 1) : Collections.emptyList();
            }
    
            // Run a breadth first search to find the correct path
            public static List<State> findPath(State start, State end) {
                Queue<List<State>> queue = new LinkedList<>();
                queue.add(Collections.singletonList(start));
    
                while (!queue.isEmpty()) {
                    List<State> currentPath = queue.poll();
                    State lastState = currentPath.get(currentPath.size() - 1);
    
                    if (lastState == end) return currentPath;
    
                    for (StateTransitionPath transition : StateTransitionPath.values()) {
                        if (transition.getStart() == lastState) {
                            List<State> newPath = new ArrayList<>(currentPath);
                            newPath.addAll(transition.getPath().subList(1, transition.getPath().size()));
                            queue.add(newPath);
                        }
                    }
                }
                return Collections.emptyList();  // No valid path found
            }
        }

    
    
        CoralHandlerSubsystem subsystem;
        CoralHandlerStateMachine(CoralHandlerSubsystem subsystem) { this.subsystem = subsystem; }
    
        public Command runStatePath(List<State> path) {
            SequentialCommandGroup ret = new SequentialCommandGroup();
    
            for (State inter : path.subList(1, path.size())) {
                ret = ret.andThen(() -> {
                    subsystem.moveElevatorTo(inter.getElevatorHeightMeters());
                    subsystem.changeArmPose(new Rotation2d((inter.getArmAngleDegrees() * Math.PI)/180));
                });
            }
            return ret;
        }
    }

    private CoralHandlerStateMachine coralHandlerStateMachine;

    public CoralHandlerSubsystem() {
        coralHandlerStateMachine = new CoralHandlerStateMachine(this);

        //Set Up Elevator Motor
        mElevatorMotor = new TalonFX(26);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorFXConfig);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorPIDConfig);
        
        mElevatorMotor.setPosition(0);
        mElevatorRequest = new PositionVoltage(0).withSlot(0);

        //Set Up Arm Motor
        mArmMotor = new TalonFX(24);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorFXConfig);
        mArmMotor.getConfigurator().apply(Robot.ctreConfigs.armMotorPIDConfig);
        
        mArmMotor.setPosition(-0.25);
        //Right is 0
        //Up is 90
        //Left is 180
        //Down is 270
        mArmRequest = new PositionVoltage(90).withSlot(0);

        mHopperSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
    }

    // Sets the height of the elevator from the bottom in meters
    public Command moveElevatorTo(double height) {
        if (0 <= height && height <= 0.762) {
            return this.run(() -> {
                mElevatorMotor.setControl(mElevatorRequest.withPosition(Conversions.metersToRotations(height, 0.13) * multiplier));
            }).alongWith(Commands.waitUntil(() -> mElevatorMotor.getVelocity().getValueAsDouble() < 0.1 && mElevatorMotor.getPosition().getValueAsDouble() < 0.02)); //0.02 meter ~=~ 0.75 inch
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
            }).alongWith(Commands.waitUntil(() -> mArmMotor.getVelocity().getValueAsDouble() < 0.1 && mArmMotor.getPosition().getValueAsDouble() < 0.05));
        } else {
            return null;
        }
    }

    public Command runStatePath(List<CoralHandlerStateMachine.State> path) {
        return coralHandlerStateMachine.runStatePath(path);
    }
}
