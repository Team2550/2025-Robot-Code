package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.Constants.CoralHandlerConstants;
import frc.robot.subsystems.CoralHandlerSubsystem.CoralHandlerStateMachine.State;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;
    private DoubleSolenoid mHopperSolenoid;
    private DoubleSolenoid mCoralGrabberSolenoid;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTable mMotorNetworkTable;
    private NetworkTableEntry mElevatorHeight;
    private NetworkTableEntry mArmAngle;
    private NetworkTableEntry mArmAngleTolCalc;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;

    private SysIdRoutine sysId;
    

    private static final double multiplier = 7.07;

    public class CoralHandlerStateMachine {
        public enum State
        {
            Rest(0.05,-90, false), // DISCOVERED
            CoralPickup1(0.05,-90, false),
            CoralPickup2(0.05,0, true),
            L1(0.05,-90, false),
            L2(0.433,-41.484, false), // DISCOVERED
            L3(0.05,-90, true),
            L4(1.2,-25, false);
    
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

            boolean getIsHopperIn() {
                return this.isHopperIn;
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
                ret = ret//.andThen(subsystem.setHopperState(inter.getIsHopperIn()))
                    .andThen(setElevatorAndArm(inter));
            }
            return ret;
        }
    }

    Rotation2d arm = new Rotation2d(-90 * (Math.PI/180), 0);

    private Command setElevatorAndArm(State inter) {
        Rotation2d armRotation2d = new Rotation2d(inter.armAngleDegrees * (Math.PI/180));
        arm = armRotation2d;
        double elevatorHeightMeters = inter.elevatorHeightMeters;
        if(inter == State.Rest){
            return Commands.sequence(
                // Move the arm to the desired position
                this.runOnce(() -> mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations()))),
                // Wait until the arm is within tolerance
                Commands.waitUntil(() -> 
                    Math.abs(mArmMotor.getVelocity().getValueAsDouble()) < 0.1 &&
                    Math.abs(mArmMotor.getPosition().getValueAsDouble() - armRotation2d.getRotations()) < 0.15
                ),

                // Move the elevator to the desired height
                this.runOnce(() -> mElevatorMotor.setControl(mElevatorRequest.withPosition(elevatorHeightMeters))),
                // Wait until the elevator is within tolerance
                Commands.waitUntil(() -> Math.abs(mElevatorMotor.getPosition().getValueAsDouble() - elevatorHeightMeters) < 0.07)
            ); 
        }else{
            return Commands.sequence(
            // Move the elevator to the desired height
            this.runOnce(() -> mElevatorMotor.setControl(mElevatorRequest.withPosition(elevatorHeightMeters))),
            // Wait until the elevator is within tolerance
            Commands.waitUntil(() -> Math.abs(mElevatorMotor.getPosition().getValueAsDouble() - elevatorHeightMeters) < 0.035),
    
            // Move the arm to the desired position
            this.runOnce(() -> mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations()))),
            // Wait until the arm is within tolerance
            Commands.waitUntil(() -> 
                Math.abs(mArmMotor.getVelocity().getValueAsDouble()) < 0.1 &&
                Math.abs(mArmMotor.getPosition().getValueAsDouble() - armRotation2d.getRotations()) < 0.15
            )
        ); }
    }
    

    private CoralHandlerStateMachine coralHandlerStateMachine;


    @Override
    public void periodic() {
        mArmAngle.setDouble(mArmMotor.getPosition().getValueAsDouble()*360);
        mElevatorHeight.setDouble(mElevatorMotor.getPosition().getValueAsDouble());
        mArmAngleTolCalc.setBoolean(Math.abs(mArmMotor.getPosition().getValueAsDouble() - arm.getRotations()) < 0.15);
    }

    public CoralHandlerSubsystem() {
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> mElevatorMotor.setControl(new VoltageOut(volts)),
                log -> log.motor("ElevatorMotor")
                            .voltage(mElevatorMotor.getSupplyVoltage().getValue())
                            .current(mElevatorMotor.getStatorCurrent().getValue())
                            .linearPosition(Meters.of(mElevatorMotor.getPosition().getValueAsDouble()))
                            .linearVelocity(MetersPerSecond.of(mElevatorMotor.getVelocity().getValueAsDouble())),
                this
            )
        );

        coralHandlerStateMachine = new CoralHandlerStateMachine(this);

        //Set Up Elevator Motor
        mElevatorMotor = new TalonFX(26);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorFXConfig);
        mElevatorMotor.getConfigurator().apply(Robot.ctreConfigs.elevatorMotorPIDConfig);
        
        mElevatorMotor.setPosition(0);
        mElevatorRequest = new PositionVoltage(0).withSlot(0);

        //Set Up Arm Motor;
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
        mCoralGrabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mMotorNetworkTable = mNetworkTable.getTable("armMotorNetworkTable");
        mElevatorHeight = mMotorNetworkTable.getEntry("ElevatorHeight");
        mArmAngle = mMotorNetworkTable.getEntry("ArmAngle");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
        mArmAngleTolCalc = mMotorNetworkTable.getEntry("giggity");
    }

    public Command runSysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command runSysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }


    public Command setHopperState(boolean isHopperIn) {
        return this.run(() -> {
            mHopperSolenoid.set(isHopperIn ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        });
    }

    public Command setCoralGrabberState(boolean isOpen) {
        return this.run(() -> {
            mCoralGrabberSolenoid.set(isOpen ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
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

    public Command runStatePath(List<CoralHandlerStateMachine.State> path) {
        return coralHandlerStateMachine.runStatePath(path);
    }
}
