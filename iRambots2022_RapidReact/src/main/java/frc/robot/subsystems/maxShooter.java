// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

public class maxShooter extends Subsystem {

  private final WPI_VictorSPX shooterMotorVictorSPX = RobotMap.shooterMotor;
  private final WPI_VictorSPX cargoMotorVictorSPX = RobotMap.cargoMotor;  

@Override
  public void initDefaultCommand() {
  }

  
  
  
  public void maxVert() {
    shooterMotorVictorSPX.set(-0.85);
    cargoMotorVictorSPX.set(0.85);
  }

public void maxDown(){
  shooterMotorVictorSPX.set(-0.01);

}
public void stop() {
  shooterMotorVictorSPX.set(0.0);
  cargoMotorVictorSPX.set(0.0);
}
}

