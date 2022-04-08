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

public class shooter extends Subsystem {

    private final WPI_VictorSPX shooterMotorVictorSPX = RobotMap.shooterMotor;
    private final WPI_VictorSPX cargoMotorVictorSPX = RobotMap.cargoMotor;
    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    @Override
    protected void initDefaultCommand() {
    
}


//shooterVert is used to make the shooter and the cargo holder run
public void shooterVert() {
    shooterMotorVictorSPX.set(-.45);
    cargoMotorVictorSPX.set(0.45);
}
//armDown makes the motor run in reverse to bring the arm down
public void shooterDown() {
    shooterMotorVictorSPX.set(-0.01);

}
public void stop() {
    shooterMotorVictorSPX.set(0.0);
    cargoMotorVictorSPX.set(0.0);
}

}