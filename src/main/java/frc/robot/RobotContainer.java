package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private Compressor mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private final SendableChooser<Command> autoChooser;
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int autoDriveAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorActionAxis = XboxController.Axis.kRightTrigger.value;

    private final int coralSetSideLeftAxis = XboxController.Axis.kLeftTrigger.value;
    private final int coralSetSideRightAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Operator Buttons
    private final JoystickButton op_ballIntakeButton =     new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton op_ballExpelButton =      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton op_climberLatchButton =   new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton op_climberUnLatchButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    private final JoystickButton op_l2Button = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton op_l3Button = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton op_l4Button = new JoystickButton(operator, XboxController.Button.kY.value);

    // Driver Buttons
    private final JoystickButton dr_climbButton =     new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton dr_unClimbButton =   new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton dr_dropCoralButton = new JoystickButton(driver, XboxController.Button.kB.value);

    // Subsystems
    private final CoralHandlerSubsystem s_CoralHandler = new CoralHandlerSubsystem();
    private final BallGrabberSubsystem s_BallGrabber =   new BallGrabberSubsystem();
    private final PhotonVision s_PhotonVision =          new PhotonVision();
    private final Climber s_Climber =                    new Climber();
    private final Swerve s_Swerve =                      new Swerve(s_PhotonVision);

    private CoralHandlerSubsystem.CoralHandlerStateMachine.State operatorSelectedCoralExpulsionState;
    private int operatorReefSideDPadSelectionPOV;

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
                () -> driver.getRawAxis(operatorActionAxis),
                () -> operator.getRawButton(XboxController.Button.kA.value),
                () -> operator.getRawButton(XboxController.Button.kX.value),
                () -> operator.getRawButton(XboxController.Button.kY.value),
                () -> operator.getPOV()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // climbButton.whileTrue(s_Climber.Climb(false));
        // climbButton.onFalse(new InstantCommand(() -> s_Climber.Stop()));
        // unClimbButton.whileTrue(s_Climber.Climb(true));
        // unClimbButton.onFalse(new InstantCommand(() -> s_Climber.Stop()));

        //grabButton.onTrue(BallGrabberSubsystem.BallGrabberCommand(s_BallGrabber));
        //driveToPoint.whileTrue(Swerve.pathfindCommand());
        // ballIntakeButton.whileTrue(null);
        // ballExpelButton.whileTrue(null);

        // DEBUG BINDINGS //
        op_ballIntakeButton.onTrue(s_BallGrabber.RunMotor(true)); // LB
        op_ballIntakeButton.onFalse(s_BallGrabber.StopMotor());
        op_ballExpelButton.onTrue(s_BallGrabber.RunMotor(false)); // RB
        op_ballExpelButton.onFalse(s_BallGrabber.StopMotor());

        op_climberLatchButton.onTrue(s_Climber.ControlPneumatics(true)); // Back (NOT DEBUG)
        op_climberUnLatchButton.onTrue(s_Climber.ControlPneumatics(false)); // Start (NOT DEBUG)

        dr_climbButton.onTrue(s_BallGrabber.controlPneumaticsCommand(true)); // Back
        dr_unClimbButton.onTrue(s_BallGrabber.controlPneumaticsCommand(false)); // Start
    
        dr_climbButton.onTrue(s_Climber.RunMotor(false));
        dr_climbButton.onFalse(s_Climber.StopMotor());
        dr_unClimbButton.onTrue(s_Climber.RunMotor(true)); 
        dr_unClimbButton.onFalse(s_Climber.StopMotor());
    
        

        // dropCoralButton.onTrue(null);
    }

    public Swerve getSwerveSubsytem() { return s_Swerve; }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
        // return null;
    }
}
