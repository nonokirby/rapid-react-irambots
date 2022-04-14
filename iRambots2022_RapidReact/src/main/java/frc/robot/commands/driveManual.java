
package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class driveManual extends Command {
        public driveManual() {
        requires(Robot.driveTrain);
    }

   
    @Override
    protected void initialize() {
      
    }



    @Override
    protected void execute() {
        
    double speed = -Robot.RobotContainer.m_driver.getRawAxis(2);
    double rotation =  Robot.RobotContainer.m_driver.getRawAxis(1);

    
    if( speed > -0.05 && speed < 0.05){
      speed = 0;
    }

    if( rotation > -0.05 && rotation < 0.05){

      rotation = 0;
    }
    if( speed < -0.5){

        rotation = 0;
      }
    Robot.driveTrain.driveArcade(-speed, rotation);

    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

    
    @Override
    protected void end() {
     
    }

    @Override
    protected void interrupted() {
    }
}
