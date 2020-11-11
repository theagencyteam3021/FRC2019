package frc.robot;

//import frc.robot.AgencySystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Drive extends AgencySystem {

    private WPI_TalonSRX rBack;
    private WPI_TalonSRX rFront;
    private WPI_TalonSRX lFront;
    private WPI_TalonSRX lBack;
    private DifferentialDrive drive;

    //Constructor for drive subsystem
    public Drive(int frontLeftID, int frontRightID, int backLeftID, int backRightID, String name, Boolean debug) {
        this.name = name;
        this.debug = debug;

        //Drive Motors
        rBack = new WPI_TalonSRX(backRightID);
        rFront = new WPI_TalonSRX(frontRightID);
        lFront = new WPI_TalonSRX(frontLeftID);
        lBack = new WPI_TalonSRX(backLeftID);

        drive = new DifferentialDrive(rFront, lFront);
        rBack.follow(rFront);
        lBack.follow(lFront);

        rFront.set(ControlMode.PercentOutput, 0);
        lFront.set(ControlMode.PercentOutput, 0);

    }

    //Drives the robot at a given speed and rotation
    //@param speed the speed to drive at
    //Precondition: -1 <= speed <= 1
    //@param rotation the rotation or turning of the robot
    //Precondition: -1 <= rotation <= 1
    public void drive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }
}
