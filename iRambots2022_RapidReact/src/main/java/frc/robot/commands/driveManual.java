
package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class driveManual extends Command {
        public driveManual() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        ////////////////////
        //Curvature Drive//
        ////////////////////
    double speed = Robot.RobotContainer.m_arcade.getRawAxis(1) - Robot.RobotContainer.m_arcade.getRawAxis(2) - -Robot.RobotContainer.m_driver.getRawAxis(2);
    double rotation = Robot.RobotContainer.m_arcade.getRawAxis(0) - Robot.RobotContainer.m_driver.getRawAxis(1);
    boolean quickTurn = speed > -0.15 && speed < 0.15;
//creates internal dead zone within code without affecting the controller
    if( speed > -0.05 && speed < 0.05){
      speed = 0;
    }
//creates internal dead zone within code without affecting the controller
    if( rotation > -0.05 && rotation < 0.05){

      rotation = 0;
    }
    if( speed < -0.5){

        rotation = 0;
      }
    Robot.driveTrain.driveCurvature(-speed, rotation, quickTurn);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
