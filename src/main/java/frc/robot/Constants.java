package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class RegistrationSafety{
        public static final boolean safetyZoneEnabled = true;
        public static final double safetyZoneMinX = -3.1244;
        public static final double safetyZoneMaxX = -1.5;
        public static final double safetyZoneMinY = 5.416;
        public static final double safetyZoneMaxY = 5.912; // TODO: FIND THE ACTUAL VALUES

        public static final double outsideZoneMultiplier = 0.1;
    }
    
    public static final class ClimbConstants 
    {
        public static final int climbMotorID = 1;
        public static final double climbSpeed = 1;
    }

    public static final class grabConstants {
        //TODO: actual id
        public static final int grabMotorID = 2;
        public static final double grabSpeed = 1;
    }

    public static final double stickDeadband = 0.1;

    public static final class vision {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        public static final String localizationCameraOneName = "Camera_Module_v1";
        public static final Transform3d localizationCameraOneToRobot = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3), //14in left of center
                Units.inchesToMeters(14), //3in behind center
                Units.inchesToMeters(12)), //12in high
            new Rotation3d(0, 
                Rotation2d.fromDegrees(0).getRadians(), 
                Rotation2d.fromDegrees(0).getRadians()));
        
        public static Matrix<N3, N1> localizationCameraOneStdDev = VecBuilder.fill(0.1, 0.1, 0.5);


        public static final String localizationCameraTwoName = "coral-side-camera";
        public static final Transform3d localizationCameraTwoToRobot = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0)),
            new Rotation3d(0, 
                Rotation2d.fromDegrees(0).getRadians(), 
                Rotation2d.fromDegrees(0).getRadians()));
        
        public static Matrix<N3, N1> localizationCameraTwoStdDev = VecBuilder.fill(0.1, 0.1, 0.5);

        public static final String localizationCameraThreeName = "algae-side-camera";
        public static final Transform3d localizationCameraThreeToRobot = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14),
                Units.inchesToMeters(-3),
                Units.inchesToMeters(12)),
            new Rotation3d(0,
                Rotation2d.fromDegrees(0).getRadians(),
                Rotation2d.fromDegrees(0).getRadians()));

        public static Matrix<N3, N1> localizationCameraThreeStdDev = VecBuilder.fill(0.1,0.1,0.5);
    }

    public static final class Swerve {
        public static final int pigeonID = 2; 

        public static final COTSTalonFXSwerveConstants chosenModule =  
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        //Will have to wait until we get the robot built
        public static final double trackWidth = Units.inchesToMeters(26.75); 
        public static final double wheelBase = Units.inchesToMeters(22.75); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        //Lock the wheels foward and run sysid as normal you would for a tank drive
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(111.44);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(51.94);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-144.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }
    }

    public static final class CoralHandlerConstants {
        public static final double restPose = 0;
        public static final double L2Pose = 0.3;
        public static final double L3Pose = 0.762;
        public static final double L4Pose = 0;  // Set this
    
        public static final Rotation2d armRestPose = new Rotation2d(-90*Math.PI/180);
        public static final Rotation2d R2Pose = new Rotation2d(-45*Math.PI/180);
      }

    //Will have to wait until we build the robot
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Reefscape {
        enum ScoringPositionID {
            LEFT_L2,
            RIGHT_L2,
            LEFT_L3,
            RIGHT_L3,
            LEFT_L4,
            RIGHT_L4
        }
        //TODO: Set to actual port numbers
        public static final int coralElevatorMotorID = -1;
        public static final int coralArmMotorID = -1;

        public static final double elevatorGearRatio = 63.5;
    }

    public static final class Controls {
        public static final class Driver {
            public static final Joystick driverJoystick = new Joystick(0);

            public static final Trigger LT_autoAlign = new Trigger(() -> driverJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7);
            public static final Trigger RT_scoringAction = new Trigger(() -> driverJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7);

            public static final JoystickButton Y_zeroGyro = new JoystickButton(driverJoystick, XboxController.Button.kY.value);
            public static final JoystickButton BACK_climb = new JoystickButton(driverJoystick, XboxController.Button.kBack.value);
            public static final JoystickButton START_unClimb = new JoystickButton(driverJoystick, XboxController.Button.kStart.value);
            public static final JoystickButton X_score = new JoystickButton(driverJoystick, XboxController.Button.kX.value);

        }

        public static final class Operator {
            public static final Joystick operatorJoystick = new Joystick(1);

            public static final Trigger LT_setScoreSideLeft = new Trigger(() -> operatorJoystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7);
            public static final Trigger RT_setScoreSideRight = new Trigger(() -> operatorJoystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7);

            public static final JoystickButton LB_ballIntake = new JoystickButton(operatorJoystick, XboxController.Button.kLeftBumper.value);
            public static final JoystickButton RB_ballExpel = new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value);
            public static final JoystickButton BACK_climberLatch = new JoystickButton(operatorJoystick, XboxController.Button.kBack.value);
            public static final JoystickButton START_climberUnLatch = new JoystickButton(operatorJoystick, XboxController.Button.kStart.value);

            public static final JoystickButton A_l2 = new JoystickButton(operatorJoystick, XboxController.Button.kA.value);
            public static final JoystickButton X_l3 = new JoystickButton(operatorJoystick, XboxController.Button.kX.value);
            public static final JoystickButton Y_l4 = new JoystickButton(operatorJoystick, XboxController.Button.kY.value);
            public static final JoystickButton B_load = new JoystickButton(operatorJoystick, XboxController.Button.kB.value);

            public static final JoystickButton RB_cancelScore = new JoystickButton(operatorJoystick, XboxController.Button.kRightBumper.value);
        }
    }
}
