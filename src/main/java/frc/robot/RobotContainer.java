package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopCoral;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.BallGrabberSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

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
    private final BallGrabberSubsystem s_BallGrabber =   new BallGrabberSubsystem();
    private final PhotonVision s_PhotonVision =          new PhotonVision();
    private final Climber s_Climber =                    new Climber();
    private final Swerve s_Swerve =                      new Swerve(s_PhotonVision);

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        mCompressor.enableDigital(); //We may want to use analog or hybrid

        autoChooser = AutoBuilder.buildAutoChooser();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> driver.getRawAxis(autoDriveAxis),
                () -> false // robot centric control
            )
        );

        s_CoralHandler.setDefaultCommand(
            new TeleopCoral(
                s_CoralHandler,
                () -> driver.getRawAxis(operatorActionAxis)
            )
        );
        
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
            zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        s_CoralHandler.configureButtonBindings();
        s_BallGrabber.configureButtonBindings();
        s_Climber.configureButtonBindings();
    }
    
    public Swerve getSwerveSubsystem() { return s_Swerve; }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
        // return null;
    }
}
