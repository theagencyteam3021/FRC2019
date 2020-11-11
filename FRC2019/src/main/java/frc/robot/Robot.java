/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Drive drive;

  private Turret turret;

  private AutonomousController autonomousController;

  XboxController DriverInputPrimary = new XboxController(0);

  Command autonomousCommand;
  SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    boolean DEBUG = true;
    drive = new Drive(RobotMap.L_DRIVE_FRONT, RobotMap.R_DRIVE_FRONT, RobotMap.L_DRIVE_BACK, RobotMap.R_DRIVE_BACK,
        "Drive", DEBUG);
    turret = new Turret(RobotMap.TURRET_ACTUATOR, RobotMap.TURRET_FEEDER, RobotMap.TURRET_SHOOTER, "Turret", DEBUG);
    autonomousController = new AutonomousController(RobotMap.POTENTIOMETER, "AutonomousController", DEBUG);
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
    drive.autonomousInit();
    turret.autonomousInit();
    autonomousController.autonomousInit();
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
    drive.teleopInit();
    turret.teleopInit();
    autonomousController.teleopInit();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    /*if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    */

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // at one point these were squared
    // threshold is 0.0075 for squared, 0.0007 for cubed
    double XboxPosY = DriverInputPrimary.getTriggerAxis(Hand.kLeft) - DriverInputPrimary.getTriggerAxis(Hand.kRight);
    double XboxPosYCubed = XboxPosY * Math.abs(XboxPosY) * Math.abs(XboxPosY);
    double XboxPosX = DriverInputPrimary.getX(Hand.kLeft); //was previsouly kRight
    double XboxPosXCubed = XboxPosX * Math.abs(XboxPosX) * Math.abs(XboxPosX);

    if (!(XboxPosYCubed > 0.0007 || XboxPosYCubed < -0.0007)) {
      XboxPosYCubed = 0;
    }
    if (!(XboxPosXCubed > 0.0007 || XboxPosXCubed < -0.0007)) {
      XboxPosXCubed = 0;
    }

    drive.drive(-XboxPosYCubed, -(XboxPosXCubed * Math.max(Math.abs(XboxPosYCubed), 0.5))); //divided by 2

    //Turret Control
    //~~~~Aiming (Raising and Lowering System)
    if (DriverInputPrimary.getYButton()) {
      turret.raiseActuator();
    } else if (DriverInputPrimary.getXButton()) {
      turret.lowerActuator();
    } else {
      turret.stopActuator();
    }

    //~~~~Shooter
    if (DriverInputPrimary.getBumper(Hand.kLeft)) {
      turret.shoot();
    } else {
      turret.stopShooting();
    }

    //~~~~Feeder
    if (DriverInputPrimary.getBumper(Hand.kRight)) {
      turret.feed();
    } else {
      turret.stopFeeder();
    }

    drive.teleopPeriodic();
    turret.teleopPeriodic();
    autonomousController.teleopPeriodic(turret.actuatorPreviouslyMoving());
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

}
