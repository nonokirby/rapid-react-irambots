 /**  package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.subsystems.driveTrain;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class auto180 extends Command {

    

    public auto180() {
        requires(Robot.driveTrain);
        setTimeout(50.0);
    }

    @Override
    protected void initialize() {
        RobotMap.ahrs.getRawGyroZ();  
        RobotMap.ahrs.reset();  
    }

    @Override
    protected void execute() {
        double time = Timer.getFPGATimestamp();
        double startTime = Timer.getMatchTime();
        
        if (time - startTime > 50.0) {
            Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0, Robot.oi.joy1.getRawAxis(0) * .6,true);
        } else {
            Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0.0, Robot.oi.joy1.getRawAxis(0) * 0.0,false);
        
        }
      
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}*/



