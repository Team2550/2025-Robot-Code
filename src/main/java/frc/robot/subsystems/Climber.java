// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// And I don't really care, I'm here to code not to do politics.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase
{
    private SparkMax m_climbMotor;
    private DoubleSolenoid m_Solenoid;
    
    public Climber()
    {
        m_climbMotor = new SparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless);
        m_Solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
    }

    @Override
    public void periodic(){}

    public Command RunMotor(boolean reverse)
    {
        return this.run(()->{
            m_climbMotor.set(ClimbConstants.climbSpeed * (reverse ? -1 : 1)); 
        });
    }

    public Command StopMotor()
    {
        return this.run(()->{
            m_climbMotor.set(0); 
        });
    }

    public Command ControlPneumatics(boolean open){
        return this.run(()->{
            if(open){
                m_Solenoid.set(DoubleSolenoid.Value.kForward);
            }else{
                m_Solenoid.set(DoubleSolenoid.Value.kReverse);
            }
        });
    }

    public Command TurnOffPneumatics(){
        return this.run(() ->{ 
            m_Solenoid.set(DoubleSolenoid.Value.kOff);
        });
    }

    public void Stop()
    {
        m_climbMotor.stopMotor();
    }

    public void configureButtonBindings() {
        Constants.Controls.Driver.BACK_climb.whileTrue(RunMotor(false));
        Constants.Controls.Driver.BACK_climb.onFalse(StopMotor());
        Constants.Controls.Driver.START_unClimb.whileTrue(RunMotor(true));
        Constants.Controls.Driver.START_unClimb.onFalse(StopMotor());
        
        Constants.Controls.Operator.BACK_climberLatch.onTrue(ControlPneumatics(true));
        Constants.Controls.Operator.START_climberUnLatch.onTrue(ControlPneumatics(false));
    }
}
