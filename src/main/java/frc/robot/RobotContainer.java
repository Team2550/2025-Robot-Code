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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CoralHandlerSubsystem.CoralHandlerStateMachine.State;

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
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Operator Buttons
    private final JoystickButton op_LB_ballIntakeButton =     new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton op_RB_ballExpelButton =      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton op_Back_climberLatchButton =   new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton op_Start_climberUnLatchButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    private final JoystickButton op_A_l2Button = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton op_X_l3Button = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton op_Y_l4Button = new JoystickButton(operator, XboxController.Button.kY.value);

    // Driver Buttons
    private final JoystickButton dr_Back_climbButton =     new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton dr_Start_unClimbButton =   new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton dr_X_dropCoralButton = new JoystickButton(driver, XboxController.Button.kX.value);

    private final Trigger dr_LT_autoAlignButton = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.7); //This essentially turns the triggers into buttons
    private final Trigger dr_RT_moveCoralButton = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.7); //This essentially turns the triggers into buttons
    // Subsystems
    private final CoralHandlerSubsystem s_CoralHandler = new CoralHandlerSubsystem();
    private final BallGrabberSubsystem s_BallGrabber =   new BallGrabberSubsystem();
    private final PhotonVision s_PhotonVision =          new PhotonVision();
    private final Climber s_Climber =                    new Climber();
    private final Swerve s_Swerve =                      new Swerve(s_PhotonVision);

    private CoralHandlerSubsystem.CoralHandlerStateMachine.State operatorSelectedCoralExpulsionState;
    private int operatorReefSideDPadSelectionPOV;
    private State queuedState;

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        queuedState = State.Rest;
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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //dr_LT_autoAlignButton.whileTrue(s_Swerve.pathfindCommand(null));

        //dr_RT_moveCoralButton.onTrue(s_CoralHandler.runStatePath(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(State.Rest,)));

        dr_Back_climbButton.whileTrue(s_Climber.RunMotor(false));
        dr_Back_climbButton.onFalse(s_Climber.StopMotor());
        dr_Start_unClimbButton.whileTrue(s_Climber.RunMotor(true));
        dr_Start_unClimbButton.onFalse(s_Climber.StopMotor());
        

        
        dr_X_dropCoralButton.onTrue(s_CoralHandler.setCoralGrabberState(false));
        dr_X_dropCoralButton.onFalse(s_CoralHandler.setCoralGrabberState(true));


        /* Operator Buttons */
        op_Y_l4Button.onTrue(s_CoralHandler.runStatePath(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(State.Rest, State.L4)));
        op_Y_l4Button.onFalse(s_CoralHandler.runStatePath(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(State.L4, State.Rest)));
        op_X_l3Button.onTrue(s_CoralHandler.runStatePath(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(State.Rest, State.L3)));//op_A_l2Button.onTrue(s_CoralHandler.SetQueuedState(CoralHandlerSubsystem.CoralHandlerStateMachine.StateTransitionPath.findPath(s_CoralHandler.getCurrentState(), State.L2)));
        //op_LB_ballIntakeButton.onTrue(); // LB
        op_LB_ballIntakeButton.onFalse(s_BallGrabber.StopMotor());
        op_LB_ballIntakeButton.whileTrue(s_BallGrabber.intakeCommand()); // LB

        op_RB_ballExpelButton.whileTrue(s_BallGrabber.RunMotor(true)); // RB
        //op_RB_ballExpelButton.onFalse();
        op_RB_ballExpelButton.onFalse(s_BallGrabber.outtakeCommand());
        //op_RB_ballExpelButton.onFalse(s_BallGrabber.controlPneumaticsCommand(true));
        
        // DEBUG BINDINGS //
        
        op_Back_climberLatchButton.onTrue(s_Climber.ControlPneumatics(true)); // Back (NOT DEBUG)
        op_Start_climberUnLatchButton.onTrue(s_Climber.ControlPneumatics(false)); // Start (NOT DEBUG)

        //dr_Back_climbButton.onTrue(s_BallGrabber.controlPneumaticsCommand(true)); // Back
        // dr_unClimbButton.onTrue(s_BallGrabber.controlPneumaticsCommand(false)); // Start

        //dr_Start_unClimbButton.onTrue(s_CoralHandler.runQueuedStatePath());
    
        dr_Back_climbButton.onTrue(s_Climber.RunMotor(false));
        dr_Back_climbButton.onFalse(s_Climber.StopMotor());
        // dr_unClimbButton.onTrue(s_Climber.RunMotor(true)); 
        // dr_unClimbButton.onFalse(s_Climber.StopMotor());
    
        

        // dropCoralButton.onTrue(null);
    }

    public Swerve getSwerveSubsystem() { return s_Swerve; }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new PathPlannerAuto("New Auto");
        return autoChooser.getSelected();
        // return null;
    }
}
