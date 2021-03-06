/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // motor IDs
  public static final int R_DRIVE_BACK = 1;
  public static final int R_DRIVE_FRONT = 2;
  public static final int L_DRIVE_FRONT = 3;
  public static final int L_DRIVE_BACK = 4;

  public static final int TURRET_ACTUATOR = 5;
  public static final int TURRET_SHOOTER = 6;
  public static final int TURRET_FEEDER = 7;

  // sensors
  public static final int POTENTIOMETER = 1;

}
