package frc.robot.commands;

import frc.robot.subsystems.Swerve;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;


public class driveToPointCommand extends Command {    
    @SuppressWarnings("unused")
    private Swerve s_Swerve;    

    public driveToPointCommand(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        
        Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        @SuppressWarnings("unused")
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );
    
    }


    @Override
    public void execute() {
    }
}