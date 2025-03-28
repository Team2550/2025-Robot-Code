package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Field2d m_Field = new Field2d();//Creates a field object to visualize the robot pose in smartdashboard. 
    public Pigeon2 gyro;

    private Field2d f = new Field2d();

    private boolean coralCamEnabled;

    private PIDController xPid = new PIDController(Constants.AutoConstants.AutoAlignkPXController,0,0); 
    private PIDController yPid = new PIDController(Constants.AutoConstants.AutoAlignkPYController,0,0); 
    private PIDController rPid = new PIDController(Constants.AutoConstants.AutoAlignkPThetaController,0,0); 

    private boolean scoringPositionIsLeft;
    private boolean scoringPositionIsL4;

    private PhotonVision m_photonVision;
    private final SwerveDrivePoseEstimator m_PoseEstimator;

    private Field2d m_field = new Field2d();


    public Swerve(PhotonVision vision) {
        coralCamEnabled = true;
        rPid.enableContinuousInput(-Math.PI, Math.PI);

        scoringPositionIsL4 = false;
        scoringPositionIsLeft=true;

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

    public void setScoreSide(boolean isLeftPost){ scoringPositionIsLeft = isLeftPost;}
    public void setScoreHeight(boolean isL4){ scoringPositionIsL4 = isL4;}

    public void driveForAutoAlign(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Get the current robot heading.
        Rotation2d heading = getHeading();
    
        // If field-relative and on the Red alliance, offset the heading by 180 degrees.
        if (fieldRelative && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            heading = heading.plus(Rotation2d.fromDegrees(180));
        }
        
        // Calculate chassis speeds based on whether we are in field-relative mode.
        ChassisSpeeds chassisSpeeds = fieldRelative 
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    heading)
            : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation);
        
        // Convert to swerve module states and desaturate speeds.
        SwerveModuleState[] swerveModuleStates =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    
        // Command each swerve module.
        for (SwerveModule mod : mSwerveMods) {
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
        coralCamEnabled = SmartDashboard.getBoolean("Coral Camera Enabled", coralCamEnabled);
        updateVisionLocalization();
        m_PoseEstimator.update(getGyroYaw(), getModulePositions());
        updateFieldView();
        Pose2d currentPose = getPose();

        // Define your fixed target poses

        Pose2d closestPose = targetPoses.get(0);
        double minDistance = currentPose.getTranslation()
                                .getDistance(closestPose.getTranslation());

        for (Pose2d targetPose : targetPoses) {
            double distance = currentPose.getTranslation()
                                .getDistance(targetPose.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = targetPose;
            }
        }

        if(!scoringPositionIsL4 && scoringPositionIsLeft){
            closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(0), new Rotation2d()));
        }else if(scoringPositionIsL4 && scoringPositionIsLeft){
            closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(1), new Rotation2d()));
        }else if(!scoringPositionIsL4 && !scoringPositionIsLeft){
            closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(2), new Rotation2d()));
        }else if(scoringPositionIsL4 && !scoringPositionIsLeft){
            closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(3), new Rotation2d()));
        }

        m_field.setRobotPose(closestPose);
        SmartDashboard.putData("FIELD STUFF", m_field);
        SmartDashboard.putNumber("reefDistance", closestPose.getX());

        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    List<Pose2d> sourcePoses = Arrays.asList(
        //Blue Sources
        new Pose2d(0.8117, 6.93, Rotation2d.fromDegrees(-54)),   
        new Pose2d(0.8117, 1.11, Rotation2d.fromDegrees(54)), 

        //Red Sources
        new Pose2d(16.7543, 6.93, new Rotation2d(Units.degreesToRadians(230))),
        //new Pose2d(16.6739, 1.0486, Rotation2d.fromDegrees(126))
        new Pose2d(16.7543, 1.11, Rotation2d.fromDegrees(126)) //16.7543 //1.11 //16.2427, 0.731

    );

    List<Pose2d> targetPoses = Arrays.asList(
        //Blue Side
        new Pose2d(3.832, 5.138, Rotation2d.fromDegrees(-60)),  // Top Left
        new Pose2d(5.179, 5.122, Rotation2d.fromDegrees(-120)), // Top Right
        new Pose2d(5.814, 4.017, Rotation2d.fromDegrees(180)), // Right
        new Pose2d(5.162, 2.895, Rotation2d.fromDegrees(120)), // Bottom Right
        new Pose2d(3.835, 2.887, Rotation2d.fromDegrees(60)), // Bottom Left
        new Pose2d(3.150, 4.025, Rotation2d.fromDegrees(0)), // Left

        //Red Side
        new Pose2d(13.734, 5.138, Rotation2d.fromDegrees(-120)),  // Top Left
        new Pose2d(12.387, 5.122, Rotation2d.fromDegrees(-60)),  // Top Right
        new Pose2d(11.752, 4.017, Rotation2d.fromDegrees(0)),    // Right
        new Pose2d(12.404, 2.895, Rotation2d.fromDegrees(60)),   // Bottom Right
        new Pose2d(13.731, 2.887, Rotation2d.fromDegrees(120)),  // Bottom Left
        new Pose2d(14.416, 4.025, Rotation2d.fromDegrees(180))   // Left
    );

    List<Translation2d> offsetPoses = Arrays.asList(
        new Translation2d(0, 0.17145-0.3048 - 0.05), //Left Pole L2 and, L3 //+0.03
        new Translation2d(-0.0508, 0.17145-0.3048 - 0.05), //Left Pole L4
        
        new Translation2d(0, -0.17145-0.3048), //Right Pole L2 and, L3
        new Translation2d(-0.0508, -0.17145-0.3048) //Right Pole L4
    );

    // public Command pathfindToClosestPoint() {
    //     // Get the current estimated pose of the robot
    //     Pose2d currentPose = getPose();

    //     // Define your fixed target poses

    //     Pose2d closestPose = targetPoses.get(0);
    //     double minDistance = currentPose.getTranslation()
    //                             .getDistance(closestPose.getTranslation());

    //     for (Pose2d target : targetPoses) {
    //         double distance = currentPose.getTranslation()
    //                             .getDistance(target.getTranslation());
    //         if (distance < minDistance) {
    //             minDistance = distance;
    //             closestPose = target;
    //         }
    //     }
        
    //     // Now return the command to pathfind to the closest pose
    //     return pidDriveToTarget(closestPose);
    // }

public Command pidDriveToTarget(Supplier<Pose2d> targetSupplier){
    return this.run(() -> { 
        turnCoralCamOff();

        f.setRobotPose(targetSupplier.get());
        SmartDashboard.putData("pidTarget", f);

        // Recalculate the target every iteration:
        Pose2d target = targetSupplier.get();
        Pose2d currentPose = getPose();

        double xOutput = xPid.calculate(currentPose.getX(), target.getX());
        double yOutput = yPid.calculate(currentPose.getY(), target.getY());
        double rOutput = rPid.calculate(
            currentPose.getRotation().getRadians(), 
            target.getRotation().getRadians()
        );

        // if(Math.abs(xOutput) < 0.0125){ xOutput = 0; }
        // if(Math.abs(yOutput) < 0.0125){ yOutput = 0; }
        // if(Math.abs(rOutput) < 0.025){ rOutput = 0; }
        
        driveForAutoAlign(new Translation2d(xOutput , yOutput ), rOutput, true, false);
    })
    // .until(() -> 
    //     xPid.calculate(getPose().getX(), targetSupplier.get().getX()) < 0.025 &&
    //     yPid.calculate(getPose().getY(), targetSupplier.get().getY()) < 0.025 &&
    //     Math.abs(getPose().getX() - targetSupplier.get().getX()) < 0.05 &&
    //     Math.abs(getPose().getY() - targetSupplier.get().getY()) < 0.005
    // )
    ;
}

    
    public Command pidDriveToTarget() {
        return this.run(()-> {
            Pose2d currentPose = getPose();

            // Define your fixed target poses

            Pose2d closestPose = targetPoses.get(0);
            double minDistance = currentPose.getTranslation()
                                    .getDistance(closestPose.getTranslation());

            for (Pose2d targetPose : targetPoses) {
                double distance = currentPose.getTranslation()
                                    .getDistance(targetPose.getTranslation());
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPose = targetPose;
                }
            }

            if(!scoringPositionIsL4 && scoringPositionIsLeft){
                closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(0), new Rotation2d()));
            }else if(scoringPositionIsL4 && scoringPositionIsLeft){
                closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(1), new Rotation2d()));
            }else if(!scoringPositionIsL4 && !scoringPositionIsLeft){
                closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(2), new Rotation2d()));
            }else if(scoringPositionIsL4 && !scoringPositionIsLeft){
                closestPose = closestPose.transformBy(new Transform2d(offsetPoses.get(3), new Rotation2d()));
            }

            double xOutput = xPid.calculate(currentPose.getX(), closestPose.getX());
            double yOutput = yPid.calculate(currentPose.getY(), closestPose.getY());
    
            // double xOutput = -(currentPose.getX() - closestPose.getX())*0.5;
            // double yOutput = -(currentPose.getY() - closestPose.getY())*0.5;
            // double rOutput = (currentPose.getRotation().getRadians() - closestPose.getRotation().getRadians()) * 0.25;
            // double rOutput = 0;
            double rOutput = rPid.calculate(
                currentPose.getRotation().getRadians(), 
                closestPose.getRotation().getRadians()
            );

            if(Math.abs(xOutput)<0.0125){xOutput=0;}
            if(Math.abs(yOutput)<0.0125){yOutput=0;}
            if(Math.abs(rOutput)<0.025){rOutput=0;}
            
            driveForAutoAlign(new Translation2d(xOutput, yOutput), rOutput, true, false);
        }); 
    }

    public Pose2d findClosestSource() {
            Pose2d currentPose = getPose();

            // Define your fixed target poses

            Pose2d closestPose = sourcePoses.get(0);
            double minDistance = currentPose.getTranslation()
                                    .getDistance(closestPose.getTranslation());

            for (Pose2d sourcePose : sourcePoses) {
                double distance = currentPose.getTranslation()
                                    .getDistance(sourcePose.getTranslation());
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPose = sourcePose;
                }
            }

            return closestPose; 
    }


    public Command turnCoralCamOn(){
        return this.runOnce(() -> {
            coralCamEnabled = true;
            SmartDashboard.putBoolean("Coral Camera Enabled", coralCamEnabled);
        });
    }

    public Command turnCoralCamOff(){
        return this.runOnce(() -> {
            coralCamEnabled = false;
            SmartDashboard.putBoolean("Coral Camera Enabled", coralCamEnabled);
        });
    }

    /*Vision Functions */
    public void updateVisionLocalization() {
        var visionEst = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraOneName);
        
        if(coralCamEnabled){
        visionEst.ifPresent(
                est -> {
                    m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraOneStdDev);
                    m_PoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds);
                });
            }
        visionEst = m_photonVision.getEstimatedGlobalPose(Constants.vision.localizationCameraTwoName);
        visionEst.ifPresent(
                est -> {
                    m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraTwoStdDev);
                    m_PoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds);
                });
    }

    public Pose2d getEstimatedPosition(){return new Pose2d();}; 
}