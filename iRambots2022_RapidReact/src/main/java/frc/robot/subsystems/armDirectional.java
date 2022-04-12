/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class armDirectional extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
private final WPI_VictorSPX directionalMotorVictorSPX = Constants.directionalMotor;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void armRight()  {
directionalMotorVictorSPX.set(-0.3);
  }
  public void armLeft()  {
directionalMotorVictorSPX.set(0.3);
  }
  public void stop() {
    directionalMotorVictorSPX.set(0.0);


  }
}
