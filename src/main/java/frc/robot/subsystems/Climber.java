// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// And I don't really care, I'm here to code not to do politics.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase
{
    private Spark m_climbMotor;
    DigitalInput limitSwitch; 
    double climbSpeed = 1;
    double timeOfClimb = 1;
    boolean isClimbing;
    boolean active;
    
    public Climber()
    {
        m_climbMotor = new Spark(ClimbConstants.climbMotorID);
        limitSwitch = new DigitalInput(-1); //This won't function until you assign the proper port
        active = true;
    }

    @Override
    public void periodic()
    {
        if (limitSwitch.get())
        {
            Deactivate();
        }
    }

    public void Climb()
    {
        if(!isClimbing & active)
        {
            Start();
        }
        else
        {
            Stop();
        }
    }

    public void Deactivate()
    {
        Stop();
        active = false;
    }

    public void Start()
    {
        if(!active) 
        {
            Stop();
            return;
        }
        m_climbMotor.set(climbSpeed);
        isClimbing = true;
    }

    public void Stop()
    {
        m_climbMotor.stopMotor();
        isClimbing = false;
    }
}
