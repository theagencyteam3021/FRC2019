/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import java.lang.*;
import java.sql.Driver;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Motor 1 Back Right Motor
  //Motor 2 Front Right Motor
  //Motor 3 Front Left Motor
  //Motor 4 Back Left Motor
  //Reverse direction of set
//Drive Motors
  WPI_TalonSRX Motor1 = new WPI_TalonSRX(1); //Back Right
  WPI_TalonSRX Motor2 = new WPI_TalonSRX(2); //Front Right
  WPI_TalonSRX Motor3 = new WPI_TalonSRX(3); //Front Left
  WPI_TalonSRX Motor4 = new WPI_TalonSRX(4); //Back Left
  //Turret Motors
  WPI_TalonSRX Motor5 = new WPI_TalonSRX(5); //Aiming (raise/lowering linear actuator)
  WPI_TalonSRX Motor6 = new WPI_TalonSRX(6); //Shooter wheel
  WPI_TalonSRX Motor7 = new WPI_TalonSRX(7); //Feeder
  int pos = 0; //for the Feeder encoder position

  DifferentialDrive diffDrive = new DifferentialDrive(Motor1, Motor3);
  
  XboxController DriverInputPrimary = new XboxController(0);

  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new DriveCommand());

    Motor2.follow(Motor1);
    Motor4.follow(Motor3);

    SmartDashboard.putData("Auto mode", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
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
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
*/

    Motor1.set(ControlMode.PercentOutput, 0);
    //Reverse motor direction later
    Motor3.set(ControlMode.PercentOutput, 0);

    Motor5.set(ControlMode.PercentOutput, 0);
    Motor6.set(ControlMode.PercentOutput, 0);
    Motor7.set(ControlMode.PercentOutput, 0);



  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    //they are cubed atm not squared

    //double XboxPosY = DriverInputPrimary.getY(Hand.kRight);
    double XboxPosY = DriverInputPrimary.getTriggerAxis(Hand.kLeft) - DriverInputPrimary.getTriggerAxis(Hand.kRight);
    double XboxPosYSquared = XboxPosY * Math.abs(XboxPosY) * Math.abs(XboxPosY);
    double XboxPosX = DriverInputPrimary.getX(Hand.kLeft); //was previsouly kRight
    double XboxPosXSquared = XboxPosX * Math.abs(XboxPosX) * Math.abs(XboxPosX);
    //System.out.println(XboxPosYSquared);
    // 0.0075 not 0.0007 for squared

     if (!(XboxPosYSquared > 0.0007 || XboxPosYSquared < -0.0007)) {
      XboxPosYSquared = 0;
     }
     if (!(XboxPosXSquared > 0.0007 || XboxPosXSquared < -0.0007)) {
      XboxPosXSquared = 0;
     }
     
     diffDrive.arcadeDrive(-XboxPosYSquared, -(XboxPosXSquared * Math.max(Math.abs(XboxPosYSquared), 0.6))); //0.5  //divided by 2

    //Turret Control
    //~~~~Aiming (Raising and Lowering System)
    if (DriverInputPrimary.getYButton()){ //Raise
      Motor5.set(0.5);//0.2
    }
    else if (DriverInputPrimary.getXButton()){ //Lower
      Motor5.set(-0.5);//0.2
    }
    else{ //Don't Move
      Motor5.set(0);
    }
    //~~~~Shooter
    if (DriverInputPrimary.getBumper(Hand.kLeft)){
      Motor6.set(-1.00);//0.75
    }
    else{
      Motor6.set(0);
    }
    //~~~~Feeder
    if (DriverInputPrimary.getBumper(Hand.kRight)){
      Motor7.set(-1);
    }
    else{
      Motor7.set(0);
    }
   }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
