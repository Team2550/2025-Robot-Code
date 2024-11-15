package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;


public class alignToTagCommand extends Command {    
    private Swerve s_Swerve;    

    public alignToTagCommand(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    public ChassisSpeeds getMoveToStuff(){ //TODO: Make this a good name. 
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0 , 0);
        double[] pose = LimelightHelpers.getBotPose_TargetSpace(Constants.limelightName);
        
        chassisSpeeds.vxMetersPerSecond = pose[0]*2.0; // Multiplying is to adjust the agressiveness. 
        if(pose[2] != 0){
            chassisSpeeds.vyMetersPerSecond = (pose[2]+2)*-2.0; // -2 is the distance that the robot will try to go to. 
        }
        chassisSpeeds.omegaRadiansPerSecond = Math.toRadians(pose[4]*3.0); 

        return chassisSpeeds;

    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = getMoveToStuff();
        Translation2d translation = new Translation2d(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        translation = new Translation2d(MathUtil.applyDeadband(translation.getX(), 0.1), MathUtil.applyDeadband(translation.getY(), 0.1));
        
        double rotation = chassisSpeeds.omegaRadiansPerSecond;
        rotation = MathUtil.applyDeadband(rotation, 1/720);

        /* Drive */
        s_Swerve.drive(
            translation, 
            rotation * Constants.Swerve.maxAngularVelocity, 
            false,
            true
        );
    }
}