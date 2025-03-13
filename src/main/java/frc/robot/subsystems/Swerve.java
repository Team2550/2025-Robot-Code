package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Field2d m_Field = new Field2d();//Creates a field object to visualize the robot pose in smartdashboard. 
    public Pigeon2 gyro;

    private PIDController xPid = new PIDController(Constants.AutoConstants.kPXController,0,0); 
    private PIDController yPid = new PIDController(Constants.AutoConstants.kPYController,0,0); 
    private PIDController rPid = new PIDController(Constants.AutoConstants.kPThetaController,0,0); 

    private Pose2d estimatedPose;

    private PhotonVision m_photonVision;
    private final SwerveDrivePoseEstimator m_PoseEstimator;

    public Swerve(PhotonVision vision) {
        m_photonVision = vision;
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(180);

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

        //TODO: Replace with PhotonVision code
        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, 
                this::setPose, 
                this::getRobotRelativeSpeeds, 
                this::driveRobotRelative, 
                new PPHolonomicDriveController(
                    new PIDConstants(Constants.AutoConstants.kPXController, 0, 0), 
                    new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0)
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
    public void updateFieldView() {
        m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", m_Field);
    }

    @Override
    public void periodic() {
        updateVisionLocalization();
        m_PoseEstimator.update(getGyroYaw(), getModulePositions());
        updateFieldView();

        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    private boolean isAtPoseSetpoint() {
        return xPid.atSetpoint() && yPid.atSetpoint() && rPid.atSetpoint();
    }

    private void setEstimatedPose(Pose2d pose) {
        this.estimatedPose = pose;
    }

    private Translation2d pointOnRotatedAxis(Translation2d pos, Rotation2d rot) {
        return pos.rotateBy(rot);
    }

    private Pose2d targetPose(double x, double y, double relX, double relY, Rotation2d rot) {
        return new Pose2d(new Translation2d(x, y).plus(new Translation2d(relX, relY).rotateBy(rot)), rot);
    }

    public Command pathfindToClosestPoint() {
        // Get the current estimated pose of the robot
        Pose2d currentPose = getPose();

        // Define your fixed target poses
        List<Pose2d> targetPoses = Arrays.asList(
            new Pose2d(3.832, 5.138, Rotation2d.fromDegrees(-60)),  // Top Left
            new Pose2d(5.179, 5.122, Rotation2d.fromDegrees(-120)), // Top Right
            new Pose2d(5.814, 4.017, Rotation2d.fromDegrees(180)), // Right
            new Pose2d(5.162, 2.895, Rotation2d.fromDegrees(120)), // Bottom Right
            new Pose2d(3.835, 2.887, Rotation2d.fromDegrees(60)), // Bottom Left
            new Pose2d(3.150, 4.025, Rotation2d.fromDegrees(0)) // Left
        );

        List<Pose2d> offsetPoses = Arrays.asList(
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)), //Left Pole L2 and, L3
            new Pose2d(1, -0.5, Rotation2d.fromDegrees(0)), //Left Pole L4
            
            new Pose2d(-1, 0, Rotation2d.fromDegrees(0)), //Right Pole L2 and, L3
            new Pose2d(-1, -0.5, Rotation2d.fromDegrees(0)) //Right Pole L4

        );

        Pose2d closestPose = targetPoses.get(0);
        double minDistance = currentPose.getTranslation()
                                .getDistance(closestPose.getTranslation());

        for (Pose2d target : targetPoses) {
            double distance = currentPose.getTranslation()
                                .getDistance(target.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = target;
            }
        }
        
        // Now return the command to pathfind to the closest pose
        return pidDriveToTarget(closestPose);
    }
    
    //TODO: Replace with PhotonVision code
    public Command pathfindCommand(Pose2d target){
        
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720)
        );
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );        

        //TODO: UNTESTED, POTENTIALLY DANGEROUS
        return pathfindingCommand.andThen(
            Commands.parallel(
                Commands.run(() -> {
                    this.estimatedPose = m_PoseEstimator.getEstimatedPosition();
                    var photonEstimatedPose = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraOneName);
                    photonEstimatedPose.ifPresent(
                        est -> {
                            setEstimatedPose(est.estimatedPose.toPose2d());
                        }
                    );
                    double xErr = xPid.calculate(estimatedPose.getX(), target.getX());
                    double yErr = yPid.calculate(estimatedPose.getY(), target.getY());
                    double rErr = rPid.calculate(estimatedPose.getRotation().getRadians(), target.getRotation().getRadians());
                    drive(new Translation2d(xErr, yErr), rErr, true, false);
                }),
                Commands.waitUntil(this::isAtPoseSetpoint)
            )
        );
    }

    public Command pidDriveToTarget(Pose2d target) {
        return this.runOnce(()-> {
            Pose2d currentPose = getPose();

            double xOutput = xPid.calculate(currentPose.getX(), target.getX());
            double yOutput = yPid.calculate(currentPose.getY(), target.getY());
            double rOutput = rPid.calculate(
                currentPose.getRotation().getRadians(), 
                target.getRotation().getRadians()
            );

            
            drive(new Translation2d(xOutput, yOutput), rOutput, true, false);
        }); 
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

        // visionEst = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraTwoName);
        // visionEst.ifPresent(
        //         est -> {
        //             m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraTwoStdDev);
        //             m_PoseEstimator.addVisionMeasurement(
        //                     est.estimatedPose.toPose2d(), est.timestampSeconds);
        //         });
    }

    public Pose2d getEstimatedPosition(){return new Pose2d();}; 
}