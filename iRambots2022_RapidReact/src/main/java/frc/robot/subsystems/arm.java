/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    public void stop() {
        m_motor.set(0.0);
    }
    //Encoder commands
    public void resetEncoder() {
        RelativeEncoder.setPosition(0);
    }
    public RelativeEncoder getArmEncoder() {
        return m_motor.getAlternateEncoder(4096)
    }
    public void moveArm(double speed){
        if(speed < 0 && getArmEncoder() <= 2500){
            //this is a test of github and wether or not it is stupid
          m_motor.set(speed);
        } else if(speed > 0 && getArmEncoder() >= -28512){
          m_motor.set(speed);
        }
          else {
              m_motor.set(0);
          }
    }// -29398.000000
}

