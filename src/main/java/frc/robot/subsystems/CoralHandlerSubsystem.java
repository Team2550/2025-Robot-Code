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
