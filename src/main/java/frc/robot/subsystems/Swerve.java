package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Field2d m_Field = new Field2d();//Creates a field object to visualize the robot pose in smartdashboard. 
    public Pigeon2 gyro;

    private photonVision m_photonVision;
    private final SwerveDrivePoseEstimator m_PoseEstimator;

    public Swerve(photonVision vision) {
        m_photonVision = vision;
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        m_PoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics, 
                getGyroYaw(), 
                getModulePositions(), 
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180)),
                    VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));

        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, 
                this::setPose, 
                this::getRobotRelativeSpeeds, 
                this::driveRobotRelative, 
                new PPHolonomicDriveController(
                    new PIDConstants(5, 0, 0), 
                    new PIDConstants(5, 0, 0)
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
                );
            }catch(Exception e){
                DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
            }
        }

    /*Drive Function */
    private void driveRobotRelative (ChassisSpeeds speeds) {
        boolean fieldRelative = false;
        //drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? speeds:speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //if(fieldRelative) {fieldRelative = false;}
        //else{fieldRelative = true; }
        
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /*Get Info Functions */
    public Pose2d getPose() {return m_PoseEstimator.getEstimatedPosition();}
    public Rotation2d getGyroYaw() {return new Rotation2d(gyro.getYaw().getValue());}
    public Rotation2d getHeading(){return getPose().getRotation();}
    public ChassisSpeeds getRobotRelativeSpeeds(){return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());}
    
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public boolean isInZone() {
        Pose2d estimatedPose = getPose();
        if(
            estimatedPose.getX() > Constants.RegistrationSafety.safetyZoneMinX && 
            estimatedPose.getX() < Constants.RegistrationSafety.safetyZoneMaxX &&
            estimatedPose.getY() > Constants.RegistrationSafety.safetyZoneMinY &&
            estimatedPose.getY() < Constants.RegistrationSafety.safetyZoneMaxY  ) {
                return true;
            } else {
                return false;
            } 
    }

    /*Setter Funtions */
    public void setPose(Pose2d pose) { m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose); }
    public void setHeading(Rotation2d heading) {
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading)); 
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) { //Used in example auto
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);   
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /*Zero/Reset Functions */
    public void zeroHeading() {
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
    
    public void resetModulesToAbsolute() {
        for(SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void updateOdometry() {
        m_PoseEstimator.update(getGyroYaw(), getModulePositions());
    }
    public void updateLocalization() {
        m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", m_Field);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PhotonVision Data", m_photonVision.getMovement());
        updateVisionLocalization();
        m_PoseEstimator.update(getGyroYaw(), getModulePositions());
        updateLocalization();

        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public static Command pathfindCommand(){
        Pose2d targetPose = new Pose2d(1.90, 7.37, Rotation2d.fromDegrees(-90));
        targetPose = new Pose2d(1.50, 5.4, Rotation2d.fromDegrees(180));
        
        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );

        return pathfindingCommand;
    }

    /*Vision Functions */
    public void updateVisionLocalization() {
        var visionEst = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraOneName);
        visionEst.ifPresent(
                est -> {
                    m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraOneStdDev);
                    m_PoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds);
                });

        visionEst = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraTwoName);
        visionEst.ifPresent(
                est -> {
                    m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraTwoStdDev);
                    m_PoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds);
                });
    }
}