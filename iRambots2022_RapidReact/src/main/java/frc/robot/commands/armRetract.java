/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class armRetract extends Command {
    //@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public armRetract() {
        requires(Robot.arm);
    }

    @Override
    protected void execute (){
        Robot.arm.armDown();
    }
    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.arm.stop();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
