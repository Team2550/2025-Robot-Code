package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;
    private DoubleSolenoid mHopperSolenoid;
    private DoubleSolenoid mCoralGrabberSolenoid;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mMotorNetworkTable;
    private NetworkTableEntry mElevatorHeight;
    private NetworkTableEntry mArmAngle;
    private NetworkTableEntry mArmAngleTolCalc;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;
    private final PositionVoltage mElevatorRequest;
    private final PositionVoltage mArmRequest;

    private SysIdRoutine sysId;
    
    Rotation2d arm = new Rotation2d(-90 * (Math.PI/180), 0);

    public enum State {
        Rest(   0.1,     0,    -90,    0),
        Intake( 1.04,  0.65, -248,   0),
        L1(     0.1,   0.05, -73.12, -53.08),
        L2(     0.309, 0.05, -74.70, -53.08),
        L3(     0.732, 0.3,  -74.70, -53.34),
        L4(     1.295, 0.75, -85, -54.23);

        double elevatorHeightMeters;
        double safeArmRotationHeightThresholdMeters;
        double readyAngleDegrees;
        double scoreAngleDegrees;

        State(double elevatorHeightMeters, double safeArmRotationHeightThresholdMeters, double readyAngleDegrees, double scoreAngleDegrees) {
            this.elevatorHeightMeters = elevatorHeightMeters;
            this.safeArmRotationHeightThresholdMeters = safeArmRotationHeightThresholdMeters;
            this.readyAngleDegrees = readyAngleDegrees;
            this.scoreAngleDegrees = scoreAngleDegrees;
        }
    }

    private State currentState;

    public Command readyArmAndElevator(State state) {
        currentState = state;
        Rotation2d armRotation2d = new Rotation2d(state.readyAngleDegrees * (Math.PI/180));
        return Commands.sequence(
            this.runOnce(() -> {
                mElevatorMotor.setControl(mElevatorRequest.withPosition(state.elevatorHeightMeters));
            }),

            new WaitUntilCommand(() -> mElevatorMotor.getPosition().getValueAsDouble() > state.safeArmRotationHeightThresholdMeters) //TODO: set to actual height
                .andThen(this.runOnce(() -> { mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations())); } ))
        );
    }

    public Command controlGripperCommand(boolean isGripped){
        return this.runOnce(() -> mCoralGrabberSolenoid.set(isGripped ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward));
    }

    private Command restArmAndElevator() {
        Rotation2d armRotation2d = new Rotation2d(State.Rest.readyAngleDegrees * (Math.PI/180));
        return Commands.sequence(
            this.runOnce(() -> {
                mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations()));
            }),

            new WaitUntilCommand(() -> mArmMotor.getPosition().getValueAsDouble() > -93 && mArmMotor.getPosition().getValueAsDouble() < -87) //TODO: set to actual height
                .andThen(this.runOnce(() -> { mElevatorMotor.setControl(mElevatorRequest.withPosition(State.Rest.elevatorHeightMeters)); } ))
        );
    }

    public Command score() {

        return Commands.sequence(
            this.runOnce(() -> {
                if (currentState != State.Intake && currentState != State.Rest) {

                    Rotation2d armRotation2d = new Rotation2d(currentState.scoreAngleDegrees * (Math.PI/180));

                    mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations()));
                    // Open the actuator
                }
            }),

            Commands.waitSeconds(0.4),


            this.runOnce(() -> {
                mCoralGrabberSolenoid.set(DoubleSolenoid.Value.kForward);
            }),
            Commands.waitSeconds(0.5),

            this.runOnce(() -> {
                if (currentState != State.Intake && currentState != State.Rest) {

                    Rotation2d armRotation2d = new Rotation2d(-90 * (Math.PI/180));

                    mArmMotor.setControl(mArmRequest.withPosition(armRotation2d.getRotations()));
                    // Open the actuator
                }
            }),
            Commands.waitSeconds(0.5),


            this.runOnce(() -> {
                mCoralGrabberSolenoid.set(DoubleSolenoid.Value.kReverse);
            })
        );
    }

    @Override
    public void periodic() {
        mArmAngle.setDouble(mArmMotor.getPosition().getValueAsDouble()*360);
        mElevatorHeight.setDouble(mElevatorMotor.getPosition().getValueAsDouble());
        mArmAngleTolCalc.setBoolean(Math.abs(mArmMotor.getPosition().getValueAsDouble() - arm.getRotations()) < 0.03);
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
        mCoralGrabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

        mNetworkTable = NetworkTableInstance.getDefault();
        mMotorNetworkTable = mNetworkTable.getTable("armMotorNetworkTable");
        mElevatorHeight = mMotorNetworkTable.getEntry("ElevatorHeight");
        mArmAngle = mMotorNetworkTable.getEntry("ArmAngle");
        mArmAngleTolCalc = mMotorNetworkTable.getEntry("giggity");
    }
    public Command runSysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command runSysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
    
    // Stops the elevator motor at its current position
    public void stopElevatorMotor() {
        mElevatorMotor.set(0);
    }

    // Stops the arm at its current position
    public void stopArmMotor(){
        mArmMotor.set(0);
    }

    public void configureButtonBindings(Swerve swerve) {

        Constants.Controls.Driver.X_score.onTrue(score());
        Constants.Controls.Driver.RB_scoringAction.onTrue(controlGripperCommand(false));
        Constants.Controls.Driver.RB_scoringAction.onFalse(controlGripperCommand(true));
        
        Constants.Controls.Operator.B_load.onTrue(readyArmAndElevator(State.Intake));
        Constants.Controls.Operator.Y_l4.onTrue(readyArmAndElevator(State.L4).alongWith(new InstantCommand(()->{swerve.setScoreHeight(true);})));
        Constants.Controls.Operator.X_l3.onTrue(readyArmAndElevator(State.L3).alongWith(new InstantCommand(()->{swerve.setScoreHeight(false);})));
        Constants.Controls.Operator.A_l2.onTrue(readyArmAndElevator(State.L2).alongWith(new InstantCommand(()->{swerve.setScoreHeight(false);})));
        // Constants.Controls.Operator.RB_cancelScore.onTrue(restArmAndElevator());
    }
}
