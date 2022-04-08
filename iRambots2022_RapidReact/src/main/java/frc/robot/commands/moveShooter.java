/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//shooter  used to be jack
public class moveShooter extends Command {
    //@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    //private boolean move;

    public moveShooter() {
        requires(Robot.shooter);
    }

    @Override
    protected void execute (){
        Robot.shooter.shooterVert();
    }
    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.shooter.stop();
    }

    @Override
    protected void interrupted() {
        end();
    }

}

