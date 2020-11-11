/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import java.lang.*;
import java.sql.Driver;

//limelight stuff
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//queues to get that hot O(1) time
import java.util.LinkedList;
import java.util.Queue;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Reverse direction of set
  //Drive Motors
  WPI_TalonSRX Motor1 = new WPI_TalonSRX(RobotMap.R_DRIVE_BACK); //Back Right
  WPI_TalonSRX Motor2 = new WPI_TalonSRX(RobotMap.R_DRIVE_FRONT); //Front Right
  WPI_TalonSRX Motor3 = new WPI_TalonSRX(RobotMap.L_DRIVE_FRONT); //Front Left
  WPI_TalonSRX Motor4 = new WPI_TalonSRX(RobotMap.L_DRIVE_BACK); //Back Left
  //Turret Motors
  WPI_TalonSRX Motor5 = new WPI_TalonSRX(RobotMap.TURRET_ACTUATOR); //Aiming (raise/lowering linear actuator)
  AnalogInput PotentiometerIn = new AnalogInput(1);
  AnalogPotentiometer pot5 = new AnalogPotentiometer(PotentiometerIn, 2578.947, 2578.947 * -0.987); // potentiometer for the motor number 5
  //output was going from about 0.987 to 1.006, needs to be from 0-49º
  //2578.947 = range of motion of shooter (49º) / range of motion of output (0.019)
  //y=2578.947(x-.987) and distribute it
  //and for some inexplicable reason it's always 3.996º less than it should be
  //move the adding 3.996 to the constructor or get rid of it altogeher
  WPI_TalonSRX Motor6 = new WPI_TalonSRX(RobotMap.TURRET_SHOOTER); //Shooter wheel
  WPI_TalonSRX Motor7 = new WPI_TalonSRX(RobotMap.TURRET_FEEDER); //Feeder

  DifferentialDrive diffDrive = new DifferentialDrive(Motor1, Motor3);

  XboxController DriverInputPrimary = new XboxController(0);

  public static OI oi;

  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  //limelight setup
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //global doubles
  double motorMotion = 0;
  double neckAngle;
  double potentiometerNeckAngle;
  boolean previousMoving = false;

  //constant heights for testing
  //note all measurements in inches
  final double TARGET_HEIGHT = 35.25;
  final double LIMELIGHT_HEIGHT = 29.75;
  final double HEIGHT_DIFFERENCE = Math.abs(TARGET_HEIGHT - LIMELIGHT_HEIGHT);
  final double CAMERA_TO_FULCRUM = 13.5;

  Queue<Double> potNeckAngleAverager = new LinkedList<Double>();
  int potNeckAngleSize = 0;
  double potNeckAngleSum = 0;
  double avgPotNeckAngle = 0;
  final int AVERAGER_MAX_SIZE = 1000;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    //chooser.setDefaultOption("Default Auto", new DriveCommand());

    Motor2.follow(Motor1);
    Motor4.follow(Motor3);

    SmartDashboard.putData("Auto mode", chooser);

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
    autonomousCommand = chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.start();
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
    //System.out.println("test 123");
    Motor1.set(ControlMode.PercentOutput, 0);
    //Reverse motor direction later
    Motor3.set(ControlMode.PercentOutput, 0);

    Motor5.set(ControlMode.PercentOutput, 0);
    Motor6.set(ControlMode.PercentOutput, 0);
    Motor7.set(ControlMode.PercentOutput, 0);

    //Reset the neck value
    motorMotion = 0;
    neckAngle = 0;
    previousMoving = false;
    Motor5.configFactoryDefault();
    // Motor5.configSelectedFeedbackSensor(FeedbackDevice.SensorSum);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //System.out.println("test 123");
    //console_debug("test123");
    Scheduler.getInstance().run();

    //they are cubed atm not squared

    // at one point these were squared
    // threshold is 0.0075 for squared, 0.0007 for cubed
    double XboxPosY = DriverInputPrimary.getTriggerAxis(Hand.kLeft) - DriverInputPrimary.getTriggerAxis(Hand.kRight);
    double XboxPosYCubed = XboxPosY * Math.abs(XboxPosY) * Math.abs(XboxPosY);
    double XboxPosX = DriverInputPrimary.getX(Hand.kLeft); //was previsouly kRight
    double XboxPosXCubed = XboxPosX * Math.abs(XboxPosX) * Math.abs(XboxPosX);
    //System.out.println(XboxPosYSquared);

    if (!(XboxPosYCubed > 0.0007 || XboxPosYCubed < -0.0007)) {
      XboxPosYCubed = 0;
    }
    if (!(XboxPosXCubed > 0.0007 || XboxPosXCubed < -0.0007)) {
      XboxPosXCubed = 0;
    }

    diffDrive.arcadeDrive(-XboxPosYCubed, -(XboxPosXCubed * Math.max(Math.abs(XboxPosYCubed), 0.5))); //divided by 2

    //Turret Control
    //~~~~Aiming (Raising and Lowering System)
    if (DriverInputPrimary.getYButton()) { //Raise
      Motor5.set(0.5);//0.2
      if (previousMoving && motorMotion < 180.0)
        motorMotion += 0.5;
      else if (motorMotion < 180.0)
        motorMotion += 0.3;
      previousMoving = true;
      //Thread.sleep(100000);
    } else if (DriverInputPrimary.getXButton()) { //Lower
      Motor5.set(-0.5);//0.2
      if (previousMoving && motorMotion > 0.0)
        motorMotion -= 0.4;
      else if (motorMotion > 0.0)
        motorMotion -= 0.2;
      previousMoving = true;
    } else { //Don't Move
      Motor5.set(0);
      previousMoving = false;
    }
    //~~~~Shooter
    if (DriverInputPrimary.getBumper(Hand.kLeft)) {
      Motor6.set(-0.75);
    } else {
      Motor6.set(0);
    }
    //~~~~Feeder
    if (DriverInputPrimary.getBumper(Hand.kRight)) {
      Motor7.set(-1);
    } else {
      Motor7.set(0);
    }

    //System.out.println("motorMotion = "+motorMotion);
    //limelight 
    //read values periodically
    double x = tx.getDouble(0.0);
    double phi = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    neckAngle = motorMotion / 4.2;

    //potentiometerNeckAngle = Math.round(pot5.get() * 1000.0) / 1000.0 + 3.996;
    potentiometerNeckAngle = pot5.get() + 3.996;
    if (!previousMoving) {
      potNeckAngleAverager.add(potentiometerNeckAngle);
      potNeckAngleSize++;
      potNeckAngleSum += potentiometerNeckAngle;
      if (potNeckAngleSize > AVERAGER_MAX_SIZE) {
        potNeckAngleSum -= potNeckAngleAverager.remove();
        potNeckAngleSize--;
        //avgPotNeckAngle = potNeckAngleSum / potNeckAngleSize;
      }
    }
    //delete queued data if moving
    else {
      potNeckAngleAverager.clear();
      potNeckAngleSize = 0;
      potNeckAngleSum = 0;
    }
    avgPotNeckAngle = potNeckAngleSum / potNeckAngleSize;

    //double netAngle = phi+potentiometerNeckAngle;
    double netAngle = phi + avgPotNeckAngle;

    //double distanceToTarget = HEIGHT_DIFFERENCE -(CAMERA_TO_FULCRUM* Math.sin(Math.toRadians(potentiometerNeckAngle)));
    double distanceToTarget = HEIGHT_DIFFERENCE - (CAMERA_TO_FULCRUM * Math.sin(Math.toRadians(avgPotNeckAngle)));
    distanceToTarget /= Math.tan(Math.toRadians(netAngle));
    //the above should be correct but dont want to mess anything up
    double distanceToTarget2 = HEIGHT_DIFFERENCE / Math.tan(Math.toRadians(phi));

    //System.out.println("xPos = "+x+" yPos = "+y+" area = "+area);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", phi);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("NeckMotion", motorMotion);
    SmartDashboard.putNumber("NeckAngle", neckAngle);
    SmartDashboard.putNumber("Distance", distanceToTarget);
    SmartDashboard.putNumber("Distance2", distanceToTarget2);
    SmartDashboard.putBoolean("WasMoving", previousMoving);
    //SmartDashboard.putNumber("NeckPos", Motor5.getSelectedSensorPosition());
    //SmartDashboard.putNumber("Velocity", Motor5.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("Out %",Motor5.getMotorOutputPercent());
    SmartDashboard.putNumber("potentiometerNeckAngle", potentiometerNeckAngle);
    SmartDashboard.putNumber("avgPotNeckAngle", avgPotNeckAngle);
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    //System.out.println(motorMotion);

  }

}
