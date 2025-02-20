package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public TalonFXConfiguration elevatorMotorFXConfig = new TalonFXConfiguration();
    public Slot0Configs elevatorMotorPIDConfig = new Slot0Configs();
    public TalonFXConfiguration armMotorFXConfig = new TalonFXConfiguration();
    public Slot0Configs armMotorPIDConfig = new Slot0Configs();

    public CTREConfigs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

        // ELEVATOR CONFIGURATION

        elevatorMotorFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        elevatorMotorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        elevatorMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorFXConfig.Feedback.SensorToMechanismRatio = 1;

        elevatorMotorPIDConfig.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorPIDConfig.kP = 0.5;
        elevatorMotorPIDConfig.kI = 0.0;
        elevatorMotorPIDConfig.kD = 0.1;
        elevatorMotorPIDConfig.kG = 0.5;

        // ARM CONFIGURATION

        armMotorFXConfig.CurrentLimits.StatorCurrentLimit = 120;
        armMotorFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        armMotorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        armMotorFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotorFXConfig.Feedback.SensorToMechanismRatio = 62.5;

        armMotorPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
        armMotorPIDConfig.kP = 24;
        armMotorPIDConfig.kI = 0.05;
        armMotorPIDConfig.kD = 0.1;
        armMotorPIDConfig.kG = -1.375;
    }
}