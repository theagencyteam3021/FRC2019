package frc.robot;

//import frc.robot.AgencySystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import java.lang.Math;

public class Turret extends AgencySystem {
    private WPI_TalonSRX shooter;
    private WPI_TalonSRX actuator;
    private WPI_TalonSRX feeder;

    private final double DISTANCE_TO_POWER_SCALE_FACTOR = 0.02; // change after testing
    private final double ANGLE_TO_POWER_SCALE_FACTOR = 0.2;

    private boolean previousMoving;

    //Constructor for drive subsystem
    public Turret(int actuatorID, int feederID, int shooterID, String name, Boolean debug) {
        this.name = name;
        this.debug = debug;
        previousMoving = false;

        //Relavant motors
        actuator = new WPI_TalonSRX(actuatorID);
        feeder = new WPI_TalonSRX(feederID);
        shooter = new WPI_TalonSRX(shooterID);

        actuator.configFactoryDefault();
        feeder.configFactoryDefault();
        shooter.configFactoryDefault();

        actuator.set(ControlMode.PercentOutput, 0);
        feeder.set(ControlMode.PercentOutput, 0);
        shooter.set(ControlMode.PercentOutput, 0);
    }

    public void teleopPeriodic() {
        shuffleDebug("WasMoving", previousMoving);
    }

    public boolean actuatorPreviouslyMoving() {
        return previousMoving;
    }

    public void raiseActuator() {
        actuator.set(0.5);
        previousMoving = true;
    }

    public void moveActuator(double angle) {
        double angleToPower = angle * ANGLE_TO_POWER_SCALE_FACTOR;
        if (angleToPower > 1.0)
            angleToPower = 1.0;
        if (angleToPower < -1.0)
            angleToPower = -1.0;
        actuator.set(angleToPower);
        previousMoving = true;
    }

    public void lowerActuator() {
        actuator.set(-0.5);
        previousMoving = true;
    }

    public void stopActuator() {
        actuator.set(0);
        previousMoving = false;
    }

    public void shoot() {
        shooter.set(-0.75);
    }

    public void shoot(double speed) {
        shooter.set(-1 * Math.abs(speed));
    }

    public void shootDistance(double distance) {
        double distanceToPower = -1 * Math.abs(distance) * DISTANCE_TO_POWER_SCALE_FACTOR;
        if (distanceToPower > 1.0)
            distanceToPower = 1.0;
        shoot(distanceToPower);
    }

    public void stopShooting() {
        shooter.set(0);
    }

    public void feed() {
        feeder.set(-1);
    }

    public void stopFeeder() {
        feeder.set(0);
    }
}
