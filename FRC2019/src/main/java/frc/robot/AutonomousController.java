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

    //distances in inches
    private final double DISTANCE_HORIZONTAL_THRESHOLD = 18.0; //change
    private final double MAX_DISTANCE_FROM_TARGET = 500.0;
    private final double MIN_DISTANCE_FROM_TARGET = 12.0;
    private double limelightYThreshold;
    private boolean autonomousAssistInProgress;

    //constant heights for testing
    //note all measurements in inches
    private final double TARGET_HEIGHT = 83.7;
    private final double LIMELIGHT_HEIGHT = 29.75;
    private final double HEIGHT_DIFFERENCE = Math.abs(TARGET_HEIGHT - LIMELIGHT_HEIGHT);
    private final double CAMERA_TO_FULCRUM = 13.5;

    private final double TARGET_VERTICAL = 17.0;
    private final double TARGET_HORIZONTAL = 37.2;
    //This may need to change

    private double camVertical;
    private double camHorizontal;
    private double targetAngle;
    private double distanceHorizontal;

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

        final double POT_SCALE_FACTOR = RANGE_OF_MOTION / (POT_UPPER_BOUND - POT_LOWER_BOUND);
        final double POT_OFFSET = POT_SCALE_FACTOR * -1.0 * POT_LOWER_BOUND;

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
        limelightHOR = table.getEntry("tlong").getDouble(0);
        limelightVER = table.getEntry("tshort").getDouble(0);

        potNeckAngleAverager = new LinkedList<Double>();
        potNeckAngleSize = 0;
        potNeckAngleSum = avgPotNeckAngle = distanceToTarget = netAngle = 0.0;
        camVertical = camHorizontal = targetAngle = distanceHorizontal = 0.0;

        autonomousAssistInProgress = false;
        limelightYThreshold = 5.0;
    }

    private void displayValues() {
        if (debug) {
            shuffleDebug("LimelightX", limelightX);
            shuffleDebug("LimelightY", limelightY);
            shuffleDebug("LimelightArea", limelightA);
            shuffleDebug("Distance", distanceToTarget);
            shuffleDebug("potentiometerNeckAngle", potentiometerNeckAngle);
            shuffleDebug("avgPotNeckAngle", avgPotNeckAngle);
            shuffleDebug("TargetAngle", targetAngle);
            shuffleDebug("DistanceToMove", distanceHorizontal);
            shuffleDebug("camHorizontal", camHorizontal);
            shuffleDebug("camVertical", camVertical);
            shuffleDebug("LimelightHOR", limelightHOR);
            shuffleDebug("LimelightVER", limelightVER);
        } else {
            shuffleDebug("DistanceToTarget", distanceToTarget);
            shuffleDebug("DistanceToMove", distanceHorizontal);
        }
    }

    private double sigmoid(double input, double maxPower, double deceleration) {
        return (2*maxPower)/(1+Math.pow(Math.E,-1*input*deceleration)) - maxPower;
    }

    private double sigmoid(double input) {
        return sigmoid(input,0.75,0.5);
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
        if (limelightY == 0.0)
            return distanceToTarget;

        distanceToTarget = HEIGHT_DIFFERENCE - (CAMERA_TO_FULCRUM * Math.sin(Math.toRadians(avgPotNeckAngle)));
        distanceToTarget /= Math.tan(Math.toRadians(netAngle));
        distanceToTarget = Math.abs(distanceToTarget);
        return distanceToTarget;
    }

    private double getTargetAngle() {
        camVertical = TARGET_VERTICAL * Math.cos(Math.toRadians(netAngle));
        camHorizontal = limelightHOR * (camVertical / limelightVER);

        //Because we're dealing with arccos here, the targetAngle could be negative.
        //FIX: Move the robot a direction, and see how the angle changes.
        targetAngle = Math.toDegrees(Math.acos(camHorizontal / TARGET_HORIZONTAL)) + limelightX;
        distanceHorizontal = distanceToTarget * Math.sin(Math.toRadians(targetAngle-limelightX));
        return (targetAngle);
    }

    public void cancelAutonomousAssist() {
        autonomousAssistInProgress = false;
    }

    public boolean autonomousAssistInProgress() {
        return autonomousAssistInProgress;
    }

    //@return double array [distance to go left/right, amount to move neck, distance from target, ready to shoot?]
    public double[] autonomousAssist() {
        double[] ans = new double[4];
        autonomousAssistInProgress = true;
        //ans[0] = distanceHorizontal; old

        //sigmoid changes distance/angle to power input
        ans[0] = sigmoid(limelightX);
        ans[1] = sigmoid(limelightY); // is this correct?
        ans[2] = sigmoid(distanceToTarget,0.75,0.117);
        ans[3] = 0.0; // 0 if not ready to shoot, 1 if ready, -1 if needs to move forward/backwards

        //two options: 1) change shooting power based on distance in shooter
        //2) change limelightY to be above the center and keep constant shooting power
        //i think #1 will be easier to implement, but we can try the other one too
        if (ans[0] <= DISTANCE_HORIZONTAL_THRESHOLD && ans[1] <= limelightYThreshold) {
            autonomousAssistInProgress = false;
            ans[3] = 1.0;
        }
        //handle these cases later
        if (ans[2] >= MAX_DISTANCE_FROM_TARGET || ans[2] <= MIN_DISTANCE_FROM_TARGET) {
            ans[3] = -1.0;
        }

        return ans;
    }

    //Updates and displays sensor values during teleop
    //@param actuatorPreviouslyMoving from turret subsystem
    public void teleopPeriodic(boolean actuatorPreviouslyMoving) {
        limelightX = table.getEntry("tx").getDouble(0);
        limelightY = table.getEntry("ty").getDouble(0);
        limelightA = table.getEntry("ta").getDouble(0);
        limelightHOR = table.getEntry("tlong").getDouble(0);
        limelightVER = table.getEntry("tshort").getDouble(0);

        getLimelightDistance(actuatorPreviouslyMoving);
        getTargetAngle();
        displayValues();
    }

}
