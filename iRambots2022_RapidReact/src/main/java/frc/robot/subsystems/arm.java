/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class arm extends Subsystem {

    private final CANSparkMax m_motor = Constants.armMotor;
    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    @Override
    protected void initDefaultCommand() {
    
}


    //armVert makes the motors run to lift up the arm 
    public void armVert() {
        m_motor.set(-1.0);
    }
    //armDown makes the motor run in reverse to bring the arm down
    public void armDown() {
        m_motor.set(1.0);
    }
    //stop makes the motor stop
    public void stop() {
        while (getArmEncoder() <= -29500){
            m_motor.set(0.5);
        }
            m_motor.set(0.0);
    }
    //Encoder commands
    public void resetEncoder() {
        m_motor.getAlternateEncoder(4096).setPosition(0);
    }
    public double getArmEncoder() {
        return m_motor.getAlternateEncoder(4096).getPosition();
    }
    public void moveArm(double speed){
        if(speed < 0 && getArmEncoder() <= 2500){
          m_motor.set(speed);
        }
          else if(speed > 0 && getArmEncoder() >= -29000){
          m_motor.set(speed);
        }
          else if (speed < 0 && getArmEncoder() <= -29500 ) {
          m_motor.set(-0.2);
        }
          else{
          m_motor.set(0);
          }
    }// -29398.000000
}

