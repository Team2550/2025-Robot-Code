// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.grabConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;

public class BallGrabberSubsystem extends SubsystemBase {
    /** Creates a new BallGrabber. */
    
    private SparkMax grabMotor;
    private DoubleSolenoid mSolenoid;

    public BallGrabberSubsystem() {
        grabMotor = new SparkMax(grabConstants.grabMotorID, MotorType.kBrushed);

        mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        mSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public Command RunMotor(boolean foward) {
        return this.run(() -> {grabMotor.set(grabConstants.grabSpeed * (foward ? 1 : -1));});
    }

    public Command StopMotor() {
        return this.run(()->{grabMotor.set(0);});
    }

    public Command controlPneumaticsCommand(boolean foward){
        return this.run(() -> {
            if(foward){
                mSolenoid.set(DoubleSolenoid.Value.kForward);
            }else{
                mSolenoid.set(DoubleSolenoid.Value.kReverse);
            }
        });
    }

    public Command turnPnuematicsOff(){
        return this.run(() -> {
            mSolenoid.set(DoubleSolenoid.Value.kOff);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
