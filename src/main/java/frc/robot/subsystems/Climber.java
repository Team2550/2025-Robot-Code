// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// And I don't really care, I'm here to code not to do politics.

package frc.robot.subsystems;

import java.security.PublicKey;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase
{
    private SparkMax m_climbMotor;
    private Solenoid m_SolenoidOne;
    private Solenoid m_SolenoidTwo; 
    double climbSpeed = 0.5;
    
    public Climber()
    {
        m_climbMotor = new SparkMax(ClimbConstants.climbMotorID, MotorType.kBrushed);
        m_SolenoidOne = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
        m_SolenoidTwo = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    }

    @Override
    public void periodic(){}

    public Command RunMotor(boolean reverse)
    {
        return this.run(()->{
            m_climbMotor.set(climbSpeed * (reverse ? -1 : 1)); 
        });
    }

    public Command ControlPneumatics(boolean open){
        return this.run(()->{
            if(open){
                m_SolenoidTwo.set(false);
                m_SolenoidOne.set(true);
            }else{
                m_SolenoidOne.set(false);
                m_SolenoidTwo.set(true);
            }
        });
    }

    public void Stop()
    {
        m_climbMotor.stopMotor();
    }
}
