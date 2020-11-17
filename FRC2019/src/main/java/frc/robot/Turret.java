package frc.robot;

//import frc.robot.AgencySystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Turret extends AgencySystem {
    private WPI_TalonSRX shooter;
    private WPI_TalonSRX actuator;
    private WPI_TalonSRX feeder;

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
