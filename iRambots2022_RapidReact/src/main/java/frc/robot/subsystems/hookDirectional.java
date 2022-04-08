// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class hookDirectional extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
private final WPI_VictorSPX hookMotorVictorSPX = RobotMap.hookMotor;
  @Override
  public void initDefaultCommand() {
  }
  public void hookRight() {
    hookMotorVictorSPX.set(-0.6);
  }
  public void hookLeft() {
    hookMotorVictorSPX.set(0.6);
  }
  public void stop() {
    hookMotorVictorSPX.set(0.0);
  }
}
