/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
//import java.util.concurrent.TimeUnit;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.command.Subsystem;
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
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("unused") public class Robot extends TimedRobot {
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
   * Change these parameters to match your setup
   */
  private static final int kCanID = 1;
  private static final MotorType kMotorType = MotorType.kBrushed;
  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 4096;

  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * An alternate encoder object is constructed using the GetAlternateEncoder() 
   * method on an existing CANSparkMax object. If using a REV Through Bore 
   * Encoder, the type should be set to quadrature and the counts per 
   * revolution set to 8192
   * If using a vex mag encoder type is quadrature and cpr is 4096
   */
  private RelativeEncoder m_alternateEncoder;




  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
 // initialize SPARK MAX with CAN ID
 m_motor = new CANSparkMax(kCanID, kMotorType);
 m_motor.restoreFactoryDefaults();

 m_alternateEncoder = m_motor.getAlternateEncoder(kAltEncType, kCPR);
 
 /**
  * In order to use PID functionality for a controller, a SparkMaxPIDController object
  * is constructed by calling the getPIDController() method on an existing
  * CANSparkMax object
  */
 m_pidController = m_motor.getPIDController();

 /**
  * By default, the PID controller will use the Hall sensor from a NEO for its
  * feedback device. Instead, we can set the feedback device to the alternate
  * encoder object
  */
 m_pidController.setFeedbackDevice(m_alternateEncoder);

 /**
  * From here on out, code looks exactly like running PID control with the 
  * built-in NEO encoder, but feedback will come from the alternate encoder
  */ 

 // PID coefficients
 kP = 0.1; 
 kI = 1e-4;
 kD = 1; 
 kIz = 0; 
 kFF = 0; 
 kMaxOutput = 1; 
 kMinOutput = -1;

 // set PID coefficients
 m_pidController.setP(kP);
 m_pidController.setI(kI);
 m_pidController.setD(kD);
 m_pidController.setIZone(kIz);
 m_pidController.setFF(kFF);
 m_pidController.setOutputRange(kMinOutput, kMaxOutput);

 // display PID coefficients on SmartDashboard
 SmartDashboard.putNumber("P Gain", kP);
 SmartDashboard.putNumber("I Gain", kI);
 SmartDashboard.putNumber("D Gain", kD);
 SmartDashboard.putNumber("I Zone", kIz);
 SmartDashboard.putNumber("Feed Forward", kFF);
 SmartDashboard.putNumber("Max Output", kMaxOutput);
 SmartDashboard.putNumber("Min Output", kMinOutput);
 SmartDashboard.putNumber("Set Rotations", 0);

    new Thread(()-> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
    }).start();
    // -31820.000000
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
    /*Command driveToPort_ = new Command() {
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

        }*//*

      }

      @Override
      protected boolean isFinished() {
        return true;
      }
    };*/

    /**Command auto180 = new Command() {
      @Override
      protected void initialize() {
        setTimeout(5.0);
        Constants.ahrs.getRawGyroZ();
        Constants.ahrs.reset();
      }

      @Override
      protected void execute() {
        //double time = Timer.getFPGATimestamp();
        //double startTime = Timer.getMatchTime();

        if (time - startTime < 0.5) {
          Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0, Robot.oi.joy1.getRawAxis(0) * .6, true);
        } else {
          Robot.driveTrain.driveCurvature(Robot.oi.joy1.getRawAxis(3) * 0.0, Robot.oi.joy1.getRawAxis(0) * 0.0, false);

        }

      }
      
      @Override
      protected boolean isFinished() {
        return true;
      }
    };
    */
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
    //SmartDashboard.putNumber("Arm Distance", Constants.armMotor.getSelectedSensorPosition());
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
     //Constants.armMotor.setSelectedSensorPosition(0);
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    

    // schedule the autonomous command (example)
    // Scheduler.getInstance().add(drive);
    Scheduler.getInstance().add(new driveManual() {
      //double timeElapsed;

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

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_alternateEncoder.getPosition());

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