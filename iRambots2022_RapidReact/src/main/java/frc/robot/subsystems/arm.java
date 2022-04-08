/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
public class arm extends Subsystem {

    private final WPI_TalonSRX armMotorVictorSPX = RobotMap.armMotor;
    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    @Override
    protected void initDefaultCommand() {
    
}


//armVert makes the motors run to lift up the arm 
public void armVert() {
    armMotorVictorSPX.set(-1.0);

}
//armDown makes the motor run in reverse to bring the arm down
public void armDown() {
    armMotorVictorSPX.set(1.0);

}
public void stop() {
    armMotorVictorSPX.set(0.0);

}

}

