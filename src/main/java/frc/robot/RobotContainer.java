package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopCoral;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.BallGrabberSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CoralHandlerSubsystem.State;

public class RobotContainer {
    private Compressor mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final SendableChooser<Command> autoChooser;
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int autoDriveAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorActionAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // Subsystems
    private final CoralHandlerSubsystem s_CoralHandler = new CoralHandlerSubsystem();
    private final BallGrabberSubsystem s_BallGrabber = new BallGrabberSubsystem();
    private final PhotonVision s_PhotonVision = new PhotonVision();
    private final Climber s_Climber = new Climber();
    private final Swerve s_Swerve = new Swerve(s_PhotonVision);

    /*
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        mCompressor.enableDigital(); // We may want to use analog or hybrid

        autoChooser = AutoBuilder.buildAutoChooser();
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver.getRawAxis(autoDriveAxis),
                        () -> false // robot centric control
                ));

        s_CoralHandler.setDefaultCommand(
                new TeleopCoral(
                        s_CoralHandler,
                        () -> driver.getRawAxis(operatorActionAxis)));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        Constants.Controls.Driver.LB_autoAlign.whileTrue(s_Swerve.pidDriveToTarget());
        Constants.Controls.Operator.LT_setScoreSideLeft.onTrue(new InstantCommand(() -> {
            s_Swerve.setScoreSide(true);
        }));
        Constants.Controls.Operator.RT_setScoreSideRight.onTrue(new InstantCommand(() -> {
            s_Swerve.setScoreSide(false);
        }));

        Constants.Controls.Driver.LT_strafeLeft.whileTrue(Commands.run(() -> {s_Swerve.drive(new Translation2d(0,Constants.Controls.Driver.driverJoystick.getRawAxis(Constants.Controls.Driver.LT_strafeLeftVal)), 0, false, false);}));
        Constants.Controls.Driver.RT_strafeRight.whileTrue(Commands.run(() -> {s_Swerve.drive(new Translation2d(0,-Constants.Controls.Driver.driverJoystick.getRawAxis(Constants.Controls.Driver.RT_strafeRightVal)), 0, false, false);}));

        s_CoralHandler.configureButtonBindings(s_Swerve);
        s_BallGrabber.configureButtonBindings();
        s_Climber.configureButtonBindings();
    }

    public Swerve getSwerveSubsystem() {
        return s_Swerve;
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        // return Commands.sequence(
        //         s_CoralHandler.readyArmAndElevator(State.L3),
        //         new InstantCommand(() -> {
        //             s_Swerve.setScoreHeight(false);
        //         }),
        //         new InstantCommand(() -> {
        //             s_Swerve.setScoreSide(false);
        //         }),
        //         s_Swerve.pidDriveToTarget().withTimeout(5),
        //         s_CoralHandler.score(),
        //         s_Swerve.pidDriveToTarget(new Pose2d(1, 1.1, new Rotation2d(Units.degreesToRadians(60))))
        //                 .withTimeout(5));
            return Commands.sequence(
            s_Swerve.turnCoralCamOff(),
            s_CoralHandler.readyArmAndElevator(State.Intake),
            Commands.waitSeconds(1),
            s_CoralHandler.controlGripperCommand(false),
            s_Swerve.pidDriveToTarget(new Pose2d(16.2927-0.05, 7.249+0.07, new Rotation2d(Units.degreesToRadians(230)))).withTimeout(5),
            s_CoralHandler.controlGripperCommand(true).withTimeout(0.5),
            s_Swerve.turnCoralCamOn(),
            s_Swerve.pidDriveToTarget(new Pose2d(15.43, 4.707, new Rotation2d(0))).withTimeout(2),
            new InstantCommand(() -> {
                s_Swerve.setScoreHeight(false);
            }),
            new InstantCommand(() -> {
                s_Swerve.setScoreSide(false);
            }),
            s_CoralHandler.readyArmAndElevator(State.L3),
            s_Swerve.pidDriveToTarget().withTimeout(5),
            s_CoralHandler.score()
            );
        // return new PathPlannerAuto("New Auto");
        // return autoChooser.getSelected();
        // return null;
    }
}
