/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX; 
//import edu.wpi.first.wpililbj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
//import frc.robot.commands.cwRoller;
//import frc.robot.commands.ccwRoller;

public class rollerSpeed extends Subsystem {
  public WPI_VictorSPX spinmotorVictorSPX = Constants.spinMotor;
  public WPI_VictorSPX rollernmotorVictorSPX = Constants.rollerMotor;
  public WPI_VictorSPX spinmotor = new WPI_VictorSPX(6);
  public WPI_VictorSPX rollermotor = new WPI_VictorSPX(6);

@Override
public void initDefaultCommand() {

}
}