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
 

public  class cargo extends Subsystem {

    private final WPI_VictorSPX cargoMotorVictorSPX = RobotMap.cargoMotor;
    private final WPI_VictorSPX shooterMotorVictorSPX = RobotMap.shooterMotor;

    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    @Override
    protected void initDefaultCommand() {
        
    }
    public void cargoUp() {
        cargoMotorVictorSPX.set(.125);
        shooterMotorVictorSPX.set(0.125);

      }
      public void putDown() {
        cargoMotorVictorSPX.set(-0.5);
       shooterMotorVictorSPX.set(0.5);
      }
      public void stop() {
        cargoMotorVictorSPX.set(0.0);
        shooterMotorVictorSPX.set(0.0);
      }
    
    }