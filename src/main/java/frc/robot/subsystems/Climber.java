// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// And I don't really care, I'm here to code not to do politics.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
//import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase
{
    private TalonFX m_climbMotor;
    DigitalInput limitSwitch; 
    double climbSpeed = 1;
    double timeOfClimb = 1;
    boolean isClimbing;
    boolean active;
    
    public Climber()
    {
        m_climbMotor = new TalonFX(ClimbConstants.climbMotorID);
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
            m_climbMotor.set(climbSpeed);
            isClimbing = true;
        }
        else
        {
            m_climbMotor.stopMotor();
            isClimbing = false;
        }
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
