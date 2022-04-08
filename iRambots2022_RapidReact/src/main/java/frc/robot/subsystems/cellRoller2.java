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
 
public class cellRoller2 extends Subsystem {

    private final WPI_VictorSPX spinMotorVictorSPX = RobotMap.spinMotor;
    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    @Override
    protected void initDefaultCommand() {
        
    }
//collectCell makes the motors run to collect the energy Cells.
    public void collectCell() {
        spinMotorVictorSPX.set(1);

      }
 
//releaseCell makes the motors run in reverse, releasing the Cells.
      public void releaseCell() {
        spinMotorVictorSPX.set(-1);

      }
	public void stop() {
        spinMotorVictorSPX.set(0.0);
	}

}
