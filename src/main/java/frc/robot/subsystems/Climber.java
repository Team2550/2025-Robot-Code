// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// And I don't really care, I'm here to code not to do politics.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase
{
    private Spark m_climbMotor;
    DigitalInput limitSwitch; 
    double climbSpeed = 0.5;
    double timeOfClimb = 1;
    boolean isClimbing = false;
    boolean active = true;
    
    public Climber()
    {
        m_climbMotor = new Spark(ClimbConstants.climbMotorID);
        limitSwitch = new DigitalInput(7); //This won't function until you assign the proper port
    }

    @Override
    public void periodic()
    {
        if (limitSwitch.get())
        {
            Deactivate();
        }
    }

    public Command Climb(boolean reverse)
    {
        /* 
        if(!isClimbing && active)
        {
            m_climbMotor.set(climbSpeed);
            isClimbing = true;
        }
        else
        {
            m_climbMotor.stopMotor();
            isClimbing = false;
        }
            */
        return this.run(()->{
            m_climbMotor.set(climbSpeed * (reverse ? -1 : 1)); 
        });
    }

    public void Deactivate()
    {
        m_climbMotor.stopMotor();
        active = false;
    }

    public void Stop()
    {
        m_climbMotor.stopMotor();
    }
}
