package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.net.SocketPermission;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor1;
    private TalonFX mElevatorMotor2;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;

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

        // CANcoder eventually probably idk their plan

        mElevatorMotor1 = new TalonFX(Constants.Reefscape.coralElevatorMotor1ID);
        mElevatorMotor1.getConfigurator().apply(new TalonFXConfiguration());

        mElevatorMotor2 = new TalonFX(Constants.Reefscape.coralElevatorMotor2ID);
        mElevatorMotor2.getConfigurator().apply(new TalonFXConfiguration());
        
        mArmMotor = new TalonFX(Constants.Reefscape.coralArmMotorID);
        mArmMotor.getConfigurator().apply(new TalonFXConfiguration());

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
    }

    public Command intake() {
        Runnable selectedHeightCommand = () -> {};
        int scoringPosition = (int)mSelectedScoringHeightEntry.getInteger(-1);
        switch (scoringPosition) {
            case 0:
            case 1:
                selectedHeightCommand = () -> changeToL2Pose();
                break;
            case 2:
            case 3:
                selectedHeightCommand = () -> changeToL3Pose();
                break;
            case 4:
            case 5:
                selectedHeightCommand = () -> changeToL4Pose();
                break;
            default:
                break;
        }
        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return this.run(selectedHeightCommand);
    }
    public Command expel() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public void changeToPickupPose() {}
    public void changeToL2Pose() {}
    public void changeToL3Pose() {}
    public void changeToL4Pose() {}
}
