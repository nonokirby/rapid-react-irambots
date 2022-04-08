/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.shooterManual;
import frc.robot.commands.moveShooter;
//import frc.robot.commands.cellOutput;
import frc.robot.commands.fullShooter;
import frc.robot.commands.cwRoller;
//import frc.robot.commands.intakeCell;
import frc.robot.commands.cargoVert;
import frc.robot.commands.putDown;
//import frc.robot.commands.ccwRoller;
import frc.robot.commands.armWest;
import frc.robot.commands.armEast;
import frc.robot.commands.armManual;
import frc.robot.commands.armRetract;
import frc.robot.commands.hookEast;
import frc.robot.commands.hookWest;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
 
 

  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

public Joystick joy1;
public Joystick joy2;
public Joystick joy3;
public JoystickButton joystickButton1;
public JoystickButton joystickButton2;
public JoystickButton joystickButton3;
public JoystickButton joystickButton4;
public JoystickButton joystickButton5;
public JoystickButton joystickButton6;
public JoystickButton joystickButton7;
public JoystickButton joystickButton8;
public JoystickButton joystickButton9;
public JoystickButton joystickButton10;
public JoystickButton joystickButton11;
public JoystickButton joystickButton12;



public OI() {

//input port 1 button reading
//input a

  joy1 = new Joystick(0);
 JoystickButton btn1a = new JoystickButton(joy1, 1);
 JoystickButton btn2a = new JoystickButton(joy1, 2);
 JoystickButton btn3a = new JoystickButton(joy1, 3);
 JoystickButton btn4a = new JoystickButton(joy1, 4);
 JoystickButton btn5a = new JoystickButton(joy1, 5);
 JoystickButton btn6a = new JoystickButton(joy1, 6);
 JoystickButton btn7a = new JoystickButton(joy1, 7);
 JoystickButton btn8a = new JoystickButton(joy1, 8);
  JoystickButton btn9a = new JoystickButton(joy1, 9);
  JoystickButton btn10a = new JoystickButton(joy1, 10);
 JoystickButton btn11a = new JoystickButton(joy1, 11);
  JoystickButton btn12a = new JoystickButton(joy1, 12);

//input port 2 button reading
//input b

  joy2 = new Joystick(1);
  JoystickButton btn1b = new JoystickButton(joy2, 1);
  JoystickButton btn2b = new JoystickButton(joy2, 2);
  JoystickButton btn3b = new JoystickButton(joy2, 3);
  JoystickButton btn4b = new JoystickButton(joy2, 4);
  JoystickButton btn5b = new JoystickButton(joy2, 5);
  JoystickButton btn6b = new JoystickButton(joy2, 6);
  JoystickButton btn7b = new JoystickButton(joy2, 7);
  JoystickButton btn8b = new JoystickButton(joy2, 8);
  JoystickButton btn9b = new JoystickButton(joy2, 9);
  JoystickButton btn10b = new JoystickButton(joy2, 10);
  JoystickButton btn11b = new JoystickButton(joy2, 11);
  JoystickButton btn12b = new JoystickButton(joy2, 12);

  joy3 = new Joystick(2);
  JoystickButton btn1c = new JoystickButton(joy3, 1);
  JoystickButton btn2c = new JoystickButton(joy3, 2);
  JoystickButton btn3c = new JoystickButton(joy3, 3);
  JoystickButton btn4c = new JoystickButton(joy3, 4);
  JoystickButton btn5c = new JoystickButton(joy3, 5);
  JoystickButton btn6c = new JoystickButton(joy3, 6);
  JoystickButton btn7c = new JoystickButton(joy3, 7);
  JoystickButton btn8c = new JoystickButton(joy3, 8);
 //JoystickButton btn9c = new JoystickButton(joy3, 9);
 //JoystickButton btn10c = new JoystickButton(joy3, 10);
 //JoystickButton btn11c = new JoystickButton(joy3, 11);
 // JoystickButton btn12c = new JoystickButton(joy3, 12);

  


  btn1a.whileHeld(new armManual());
  btn2a.whileHeld(new armRetract());
  btn3a.whileHeld(new hookEast());
  btn4a.whileHeld(new hookWest());
  btn5a.whileHeld(new shooterManual());
  btn6a.whileHeld(new fullShooter());
  btn7a.whileHeld(new armWest());
  btn8a.whileHeld(new armEast());
  btn9a.whileHeld(new putDown());
  btn10a.whileHeld(new putDown());
  btn11a.whileHeld(new armManual());
  btn12a.whileHeld(new armRetract());

//input port 2 button mapping

  btn1b.whileHeld(new cargoVert());
  //btn1b.whileHeld(new intakeCell());
  btn2b.whileHeld(new armManual());
  btn3b.whileHeld(new shooterManual());
  btn4b.whileHeld(new putDown());
  btn5b.whileHeld(new fullShooter());
  btn6b.whileHeld(new putDown());
  btn7b.whileHeld(new hookEast());
  btn8b.whileHeld(new hookWest());
  btn9b.whileHeld(new armWest());
  btn10b.whileHeld(new armEast());
  btn11b.whileHeld(new armManual());
  btn12b.whileHeld(new armRetract());

  btn1c.whileHeld(new cargoVert());
  //btn1c.whileHeld(new intakeCell());
  btn2c.whileHeld(new armManual());
  btn3c.whileHeld(new armWest());
  btn4c.whileHeld(new hookWest());
  btn5c.whileHeld(new hookEast());
  btn6c.whileHeld(new shooterManual());
  btn7c.whileHeld(new armRetract());
  btn8c.whileHeld(new armEast());

}

public Joystick getjoy1() {
  return joy1;
}
public Joystick getjoy2() {
  return joy2;
};

//public GenericHID getJoystick() {
	//return null;
//}

//public Object interpretHatState(Object joystick1, int i, int j, double d, double e) {
	//return null;
}

