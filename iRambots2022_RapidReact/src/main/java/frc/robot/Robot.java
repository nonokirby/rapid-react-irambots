/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooterManual;
//import frc.robot.commands.cellOutput;
import frc.robot.commands.driveManual;
import frc.robot.commands.armManual;
//import frc.robot.commands.intakeCell;
import frc.robot.commands.cargoVert;
import frc.robot.commands.putDown;
import frc.robot.commands.armEast;
import frc.robot.commands.armWest;
import frc.robot.commands.hookWest;
import frc.robot.commands.hookEast;
//import frc.robot.commands.ccwRoller;
//import frc.robot.commands.moveShooter;
import frc.robot.commands.autonomous.DriveToPort;
//import frc.robot.commands.autonomous.TurnAround;
//import frc.robot.commands.autonomous.auto180;
//import frc.robot.commands.autonomous.autonomous;
import frc.robot.subsystems.arm;
//import frc.robot.subsystems.cellRoller;
//import frc.robot.subsystems.cellRoller2;
import frc.robot.subsystems.driveTrain;
import frc.robot.subsystems.hookDirectional;
import frc.robot.subsystems.cargo;
import frc.robot.subsystems.rollerSpeed;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.maxShooter;
import frc.robot.subsystems.armDirectional;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static driveTrain driveTrain;
  public static SequentialCommandGroup autonomousForward;
  // private static double autoStart;
  // What's the problem with autoStart?
  // nothing now, cant have a problem if it isnt a part of the running code
  //public static cellOutput cellOutput;
  //public static intakeCell intakeCell;
  public static armWest armWest;
  public static armEast armEast;
  public static hookEast hookEast;
  public static hookWest hookWest;
  public static cargoVert cargoVert;
  //public static cellRoller cellRoller;
  public static armManual armManual;
  public static frc.robot.subsystems.armDirectional armDirectional;
  public static frc.robot.subsystems.hookDirectional hookDirectional;
  //public static Command cargoVert;
  //public static cellRoller2 cellRoller2;
  public static cargo cargo;
  public static shooterManual shooterManual;
  public static shooter shooter;
  public static putDown putDown;
  public static RobotContainer RobotContainer;
  public static arm arm;
  public static Command m_autonomousCommand;
  public static DriveToPort DriveToPort;
  public static maxShooter maxShooter;
  //public static TurnAround turnAround;
  //public static auto180 auto180;
  public static Object wheelShooter;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static rollerSpeed rollerSpeed = new rollerSpeed();
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    new Thread(()-> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    }).start();
    //TODO -31820.000000
    Constants.init();
    driveTrain = new driveTrain();
    shooter = new shooter();
    //cellRoller = new cellRoller();
    cargo = new cargo();
   // cellRoller2 = new cellRoller2();
    rollerSpeed = new rollerSpeed();
    arm = new arm();
    armDirectional = new armDirectional();
   hookDirectional = new hookDirectional();
    maxShooter = new maxShooter();
    RobotContainer = new RobotContainer();

    
    // m_chooser.addDefault("Default Auto", new autonomous());
    // chooser.addObject("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    autonomousForward = new SequentialCommandGroup();
    Command driveToPort_ = new Command() {
      @Override
      protected void initialize() {
        setTimeout(5.0);
      }

      @Override
      protected void execute() {
        double time = Timer.getFPGATimestamp();
        double startTime = Timer.getMatchTime();

        /*if (time - startTime < 5.0) {
          Robot.driveTrain.driveCurvature(-Robot.oi.joy1.getRawAxis(3) * 0.3, Robot.oi.joy1.getRawAxis(0) * 0.3, true);
        } else {
          Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0.0, Robot.oi.joy1.getRawAxis(0) * 0.0, false);

        }*/

      }

      @Override
      protected boolean isFinished() {
        return true;
      }
    };

    Command auto180 = new Command() {
      @Override
      protected void initialize() {
        setTimeout(5.0);
        Constants.ahrs.getRawGyroZ();
        Constants.ahrs.reset();
      }

      @Override
      protected void execute() {
        double time = Timer.getFPGATimestamp();
        double startTime = Timer.getMatchTime();

        /*if (time - startTime < 0.5) {
          Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0, Robot.oi.joy1.getRawAxis(0) * .6, true);
        } else {
          Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0.0, Robot.oi.joy1.getRawAxis(0) * 0.0, false);

        }*/

      }

      @Override
      protected boolean isFinished() {
        return true;
      }
    };

    // autonomousForward.addCommands(new Command());
    m_autonomousCommand = new DriveToPort();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Distance", Constants.armMotor.getSelectedSensorPosition());
  }
 
  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
     m_autonomousCommand = m_chooser.getSelected();
     Constants.armMotor.setSelectedSensorPosition(0);
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    

    // schedule the autonomous command (example)
    // Scheduler.getInstance().add(drive);
    Scheduler.getInstance().add(new driveManual() {
      double timeElapsed;

      @Override
      protected void initialize() {
         driveTrain.tankDrive(-.6, .6);
        Constants.shooterMotor.set(-0.7);
        Constants.cargoMotor.set(0.7);
        
      }



      @Override
      protected void execute() {
        double startTime = Timer.getMatchTime();
        double time = Timer.getFPGATimestamp();
        if (startTime - time == 13) {
           driveTrain.tankDrive(0.0, 0.0);
           Constants.shooterMotor.set(0);
           Constants.cargoMotor.set(0);
        }
         
         
        
  
    };
  
      
        
        
        //timeElapsed = Timer.getFPGATimestamp();
        //while (timeElapsed < 1 ||timeElapsed > 10){
        //  driveTrain.tankDrive(+0.5, -0.5);
          
        //;
        

        @Override
        protected boolean isFinished() {

          double startTime = Timer.getMatchTime();
          if (startTime == 9) {
            driveTrain.tankDrive(0.0, 0.0);
            shooter.stop();
            return true;
          }
          else {
            return false;
          }
         
        }
      });
    }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
    driveTrain.tankDrive(0.0, 0.0);
    Constants.shooterMotor.set(0.0);
    Constants.cargoMotor.set(0.0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  } 

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void SequentialCommandGroup() {

  }

}