package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//limelight stuff
import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//queues to get that hot O(1) time
import java.util.LinkedList;
import java.util.Queue;

import java.lang.Math;

public class AutonomousController extends AgencySystem {
    private AnalogInput potentiometerInput;
    private AnalogPotentiometer actuatorPotentiometer;
    private NetworkTable table;

    private double limelightX;
    private double limelightY;
    private double limelightA;
    private double limelightHOR;
    private double limelightVER;

    private double potentiometerNeckAngle;
    private double distanceToTarget;
    private double netAngle;

    //constant heights for testing
    //note all measurements in inches
    private final double TARGET_HEIGHT = 35.25;
    private final double LIMELIGHT_HEIGHT = 29.75;
    private final double HEIGHT_DIFFERENCE = Math.abs(TARGET_HEIGHT - LIMELIGHT_HEIGHT);
    private final double CAMERA_TO_FULCRUM = 13.5;

    private final double TARGET_VERTICAL = 17.0;
    private final double TARGET_HORIZONTAL = 39.75;

    private double camVertical;
    private double camHorizontal;
    private double targetAngle;
    private double distanceToMove;

    private Queue<Double> potNeckAngleAverager;
    private int potNeckAngleSize;
    private double potNeckAngleSum;
    private double avgPotNeckAngle;
    private final int AVERAGER_MAX_SIZE = 50;

    private final double RANGE_OF_MOTION = 47.5;
    private final double POT_UPPER_BOUND = 1.00505;
    private final double POT_LOWER_BOUND = 0.9865;
    public AutonomousController(int potentiometerID, String name, boolean debug) {
        this.name = name;
        this.debug = debug;

        final double POT_SCALE_FACTOR = RANGE_OF_MOTION / (POT_UPPER_BOUND-POT_LOWER_BOUND);
        final double POT_OFFSET = POT_SCALE_FACTOR*-1.0*POT_LOWER_BOUND;

        potentiometerInput = new AnalogInput(potentiometerID);
        actuatorPotentiometer = new AnalogPotentiometer(potentiometerInput, POT_SCALE_FACTOR, POT_OFFSET); // potentiometer for the motor number 5
        //actuatorPotentiometer = new AnalogPotentiometer(potentiometerInput, 1, 0);
        //output was going from about 0.987 to 1.006, needs to be from 0-49º
        //2578.947 = range of motion of shooter (49º) / range of motion of output (0.019)
        //y=2578.947(x-.987) and distribute it
        //and for some inexplicable reason it's always 3.996º less than it should be
        //move the adding 3.996 to the constructor or get rid of it altogeher

        table = NetworkTableInstance.getDefault().getTable("limelight");
        limelightX = table.getEntry("tx").getDouble(0);
        limelightY = table.getEntry("ty").getDouble(0);
        limelightA = table.getEntry("ta").getDouble(0);
        limelightHOR = table.getEntry("thor").getDouble(0);
        limelightVER = table.getEntry("tver").getDouble(0);


        potNeckAngleAverager = new LinkedList<Double>();
        potNeckAngleSize = 0;
        potNeckAngleSum = 0;
        avgPotNeckAngle = 0;
        distanceToTarget = 0;
        netAngle = 0;
    }

    private void displayValues() {
        shuffleDebug("LimelightX", limelightX);
        shuffleDebug("LimelightY", limelightY);
        shuffleDebug("LimelightArea", limelightA);
        shuffleDebug("Distance", distanceToTarget);
        shuffleDebug("potentiometerNeckAngle", potentiometerNeckAngle);
        shuffleDebug("avgPotNeckAngle", avgPotNeckAngle);
        shuffleDebug("TargetAngle", targetAngle);
        shuffleDebug("DistanceToMove", distanceToMove);
    }

    //Gets distance to target according to limelight
    //@param actuatorPreviouslyMoving from turret subsystem
    //boolean is used to optimize the queue and make sure the rolling average
    //doesn't get too delayed when unnecessary
    private double getLimelightDistance(boolean actuatorPreviouslyMoving) {
        limelightX = table.getEntry("tx").getDouble(0);
        limelightY = table.getEntry("ty").getDouble(0);
        limelightA = table.getEntry("ta").getDouble(0);
        potentiometerNeckAngle = actuatorPotentiometer.get(); 
        if (!actuatorPreviouslyMoving) {
            potNeckAngleAverager.add(potentiometerNeckAngle);
            potNeckAngleSize++;
            potNeckAngleSum += potentiometerNeckAngle;
            if (potNeckAngleSize > AVERAGER_MAX_SIZE) {
                potNeckAngleSum -= potNeckAngleAverager.remove();
                potNeckAngleSize--;
            }
            avgPotNeckAngle = potNeckAngleSum / potNeckAngleSize;
        }
        //delete queued data if moving
        else {
            potNeckAngleAverager.clear();
            potNeckAngleSize = 0;
            potNeckAngleSum = 0;
        }

        netAngle = limelightY + avgPotNeckAngle;
        if (limelightY == 0.0) return Math.abs(distanceToTarget);

        distanceToTarget = HEIGHT_DIFFERENCE - (CAMERA_TO_FULCRUM * Math.sin(Math.toRadians(avgPotNeckAngle)));
        distanceToTarget /= Math.tan(Math.toRadians(netAngle));
        return Math.abs(distanceToTarget);
    }

    private double getTargetAngle() {
        camVertical = TARGET_VERTICAL * Math.cos(Math.toRadians(netAngle));
        camHorizontal = limelightHOR * (limelightVER / camVertical);

        //Because we're dealing with arccos here, the targetAngle could be negative.
        //FIX: Move the robot a direction, and see how the angle changes.
        targetAngle = Math.toDegrees(Math.acos(camHorizontal/TARGET_HORIZONTAL)+limelightX);
        distanceToMove = distanceToTarget*Math.cos(Math.toRadians(targetAngle));
        return(targetAngle);
    }

    //Updates and displays sensor values during teleop
    //@param actuatorPreviouslyMoving from turret subsystem
    public void teleopPeriodic(boolean actuatorPreviouslyMoving) {
        limelightX = table.getEntry("tx").getDouble(0);
        limelightY = table.getEntry("ty").getDouble(0);
        limelightA = table.getEntry("ta").getDouble(0);
        limelightHOR = table.getEntry("thor").getDouble(0);
        limelightVER = table.getEntry("tver").getDouble(0);

        getLimelightDistance(actuatorPreviouslyMoving);
        getTargetAngle();
        displayValues();
    }

}
