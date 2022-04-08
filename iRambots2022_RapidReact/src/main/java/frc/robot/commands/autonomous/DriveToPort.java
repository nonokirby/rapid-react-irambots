package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;
//import frc.robot.subsystems.driveTrain;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class DriveToPort extends Command {

    public DriveToPort () {
        requires(Robot.driveTrain);
        setTimeout(5.0);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        double startTime = Timer.getMatchTime();
        //TODO change joystick mapping for drivetrain here
        if ( startTime < 6.0) { 
            Robot.driveTrain.driveCurvature(-Robot.oi.joy1.getRawAxis(3)* 0.7, Robot.oi.joy1.getRawAxis(0) * 0.7,true);
        } else {
            Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0.0, Robot.oi.joy1.getRawAxis(0) * 0.0,false);
            
        }
      
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}


