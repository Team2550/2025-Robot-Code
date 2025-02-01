package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.reflect.Array;
import java.net.SocketPermission;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class CoralHandlerSubsystem extends SubsystemBase {
    
    private TalonFX mElevatorMotor;
    private TalonFX mArmMotor;

    private NetworkTableInstance mNetworkTable;
    private NetworkTable mScoringNetworkTable;
    private NetworkTableEntry mSelectedScoringHeightEntry;
    private NetworkTableEntry mSelectedScoringSideEntry;

    public CoralHandlerSubsystem() {

        // CANcoder eventually probably idk their plan

        Slot0Configs elevatorMotorConfig = new Slot0Configs();
        elevatorMotorConfig.kP = 2.4;
        elevatorMotorConfig.kI = 0;
        elevatorMotorConfig.kD = 0.1;

        mElevatorMotor = new TalonFX(Constants.Reefscape.coralElevatorMotorID);
        mElevatorMotor.getConfigurator().apply(elevatorMotorConfig);
        
        mArmMotor = new TalonFX(Constants.Reefscape.coralArmMotorID);
        mArmMotor.getConfigurator().apply(new TalonFXConfiguration());

        mNetworkTable = NetworkTableInstance.getDefault();
        mScoringNetworkTable = mNetworkTable.getTable("automaticScoringPosition");
        mSelectedScoringHeightEntry = mScoringNetworkTable.getEntry("scoringHeight");
        mSelectedScoringSideEntry = mScoringNetworkTable.getEntry("scoringSide");
    }

    public Command autoPickup() {
        return null;
    }

    public Command autoExpel() {
        return null;
    }

    public Command intake() {
        Command selectedHeightCommand = null;
        int scoringPosition = (int)mSelectedScoringHeightEntry.getInteger(-1);
        switch (scoringPosition) {
            case 0:
            case 1:
                selectedHeightCommand = changeToL2Pose();
                break;
            case 2:
            case 3:
                selectedHeightCommand = changeToL3Pose();
                break;
            case 4:
            case 5:
                selectedHeightCommand = changeToL4Pose();
                break;
            default:
                break;
        }
        //TODO: Check if doing automatic scoring and obtain scoring position if so
        return selectedHeightCommand.alongWith(null);
    }
    public Command expel() { return null; }

    //TODO: Probably come up with better naming to not confuse with the Swerve system and Pose2D's
    public Command changeToPickupPose() {
        return null;
    }
    // 50:1 24:1
    public Command changeToL2Pose() {
        mElevatorMotor.setPosition(Conversions.metersToRotations(0.25, 0.05));
        //TODO: Testing Required
        double gearRatio = 62.5;
        mArmMotor.setPosition(0.25 * gearRatio);
        return null;
    }
    public Command changeToL3Pose() {
        return null;
    }
    public Command changeToL4Pose() {
        return null;
    }
}
