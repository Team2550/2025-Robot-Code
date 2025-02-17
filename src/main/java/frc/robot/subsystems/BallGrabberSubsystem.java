// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.grabConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class BallGrabberSubsystem extends SubsystemBase {
    /** Creates a new BallGrabber. */
    
    private SparkMax grabMotor;
    private double grabSpeed;
    private boolean grab;

    public BallGrabberSubsystem() {
        this.grabMotor = new SparkMax(grabConstants.grabMotorID, MotorType.kBrushed);
        this.grabSpeed = grabConstants.grabSpeed;
        this.grab = true;
    }

    public void GrabReleaseBall() {
        if (grab) {
            grabMotor.set(grabSpeed);
        } else if (!grab) {
            grabMotor.set(-grabSpeed);
        }
    }

    public void StopGrabber() {
        grabMotor.set(0);
        grab = !grab;
    }

    public static Command BallGrabberCommand(BallGrabberSubsystem ballGrabber) {
        return Commands.runEnd(() -> ballGrabber.GrabReleaseBall(), () -> ballGrabber.StopGrabber());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
