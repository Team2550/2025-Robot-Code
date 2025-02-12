package frc.robot.subsystems;

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
        

    State currentState;
    enum State 
    {
        Rest,
        CoralPickup1,
        CoralPickup2,
        MoveHopper,
        L1,
        L2,
        L3,
        L4
    }

    public State GetCurrentState()
    {
        return currentState;
    }

    public void HandleEvent(State event)
    {
        switch (currentState) {
            case Rest:
                if(event == State.CoralPickup1)
                    currentState = State.CoralPickup1;
                if(event == State.MoveHopper)
                    currentState = State.MoveHopper;
                if(event == State.L3)
                    currentState = State.L3;
                if(event == State.L4)
                    currentState = State.L4;
                break;
            case CoralPickup1:
                if(event == State.CoralPickup2)
                    currentState = State.CoralPickup2;
                if (event == State.Rest)
                    currentState = State.Rest;
                break;
            case CoralPickup2:
                if(event == State.CoralPickup1)
                    currentState = State.CoralPickup1;
                break;
            case MoveHopper:
                if(event == State.L1)
                    currentState = State.L1;
                if(event == State.L2)
                    currentState = State.L2;
                    break;
            case L1:
                if(event == State.MoveHopper)
                    currentState = State.MoveHopper;
                break;
            case L2:
                if(event == State.MoveHopper)
                    currentState = State.MoveHopper;
                break;
            case L3:
                if(event == State.Rest)
                    currentState = State.Rest;
                break;
            case L4:
                if(event == State.Rest)
                    currentState = State.Rest;
                break;
            default:
                break;
        }
    }


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
        switch (scoringPosition) {
            case 0:
            case 1:
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L2Pose);
                break;
            case 2:
            case 3:
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L3Pose);
                break;
            case 4:
            case 5:
                selectedHeightCommand = () -> moveElevatorTo(CoralHandlerConstants.L4Pose);
                break;
            default:
                break;
        }
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
