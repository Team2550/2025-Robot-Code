package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> scoringHeightChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driveToPoint = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton climbButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton grabButton = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final photonVision s_PhotonVision = new photonVision(Constants.vision.localizationCameraOneName, Constants.vision.localizationCameraTwoName);
    private final Swerve s_Swerve = new Swerve(s_PhotonVision);
    private final CoralHandlerSubsystem s_CoralHandler = new CoralHandlerSubsystem();
    private final Climber s_Climber = new Climber();
    private final BallGrabber s_BallGrabber = new BallGrabber();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        );

        scoringHeightChooser = new SendableChooser<Integer>();

        // yes its ugly
        scoringHeightChooser.addOption("Left L2", Constants.Reefscape.ScoringPositionID.LEFT_L2.ordinal());
        scoringHeightChooser.addOption("Right L2", Constants.Reefscape.ScoringPositionID.RIGHT_L2.ordinal());
        scoringHeightChooser.addOption("Left L3", Constants.Reefscape.ScoringPositionID.LEFT_L3.ordinal());
        scoringHeightChooser.addOption("Right L3", Constants.Reefscape.ScoringPositionID.RIGHT_L3.ordinal());
        scoringHeightChooser.addOption("Left L4", Constants.Reefscape.ScoringPositionID.LEFT_L4.ordinal());
        scoringHeightChooser.addOption("Right L4", Constants.Reefscape.ScoringPositionID.RIGHT_L4.ordinal());

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Scoring Height Chooser", scoringHeightChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    public Swerve getSwerveSubsytem() { return s_Swerve; }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        climbButton.onTrue(new InstantCommand(() -> s_Climber.Climb()));
        grabButton.onTrue(BallGrabber.BallGrabberCommand(s_BallGrabber));
        driveToPoint.whileTrue(Swerve.pathfindCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();

    }
}
