/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.commands.driveManual;



public class driveTrain extends Subsystem {
 


  private final MotorControllerGroup leftMotors = Constants.driveTrainLeftMotors;
  private final MotorControllerGroup rightMotors = Constants.driveTrainRightMotors;
  
  private final DifferentialDrive differentialDrive = Constants.driveTrainDifferentialDrive;

public Object driveArcade;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new driveManual());
  }

  public MotorControllerGroup getLeftMotors() {
    return leftMotors;
  }

  public MotorControllerGroup getRightMotors(){
    return rightMotors;
  }

  public void driveTank(final double left, final double right) {
    differentialDrive.tankDrive(left, right);
  }
  
  public void driveArcade(final double speed, final double rotation) {
    differentialDrive.arcadeDrive(speed, rotation);
  }

   
 

  public void stop() {
    differentialDrive.stopMotor();
  }

public static Object getInstance() {

  
	return null;
}


public void arcadeDrive(double speed, double rotation) {
  differentialDrive.arcadeDrive(speed, rotation);
}

public void tankDrive(double left, double right) {
  differentialDrive.tankDrive(left, right);
}

}







