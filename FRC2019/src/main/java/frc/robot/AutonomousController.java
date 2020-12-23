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
    private double limelightV;
    private double limelightS;

    private double potentiometerNeckAngle;
    private double distanceToTarget;
    private double netAngle;

    //distances in inches
    private final double LIMELIGHT_Y_OFFSET = -12.0;
    private final double LIMELIGHT_X_Y_THRESHOLD = 0.35;
    private final double ANGLE_THRESHOLD = 5.0;
   // private final double MAX_DISTANCE_FROM_TARGET = 500.0;
   // private final double MIN_DISTANCE_FROM_TARGET = 12.0;
    private boolean autonomousAssistInProgress;

    //constant heights for testing
    //note all measurements in inches
    private final double TARGET_HEIGHT = 83.7;//83.7 for outside target
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

    private double checkAngleSignIterations = 0;
    private double angleSign = 0;
    private double cachedAngle = 0;

    private double turnPower;
    private double drivePower;


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
    }

    private void displayValues() {
        if (debug) {
            shuffleDebug("LimelightV", limelightV);
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
            shuffleDebug("IsAssisting", autonomousAssistInProgress);
            shuffleDebug("LimelightS", limelightS);
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
        if (limelightV == 0.0)
            return distanceToTarget;

        distanceToTarget = HEIGHT_DIFFERENCE - (CAMERA_TO_FULCRUM * Math.sin(Math.toRadians(avgPotNeckAngle)));
        distanceToTarget /= Math.tan(Math.toRadians(netAngle));
        distanceToTarget = Math.abs(distanceToTarget);
        return distanceToTarget;
    }

    /*private double getTargetAngle() {
        camVertical = TARGET_VERTICAL * Math.cos(Math.toRadians(netAngle));
        camHorizontal = limelightHOR * (camVertical / limelightVER);

        //Because we're dealing with arccos here, the targetAngle could be negative.
        //FIX: not this Move the robot a direction, and see how the angle changes.
        double horizontalRatio = Math.min(camHorizontal / TARGET_HORIZONTAL,1.0);
        targetAngle = Math.toDegrees(Math.acos(horizontalRatio)) + limelightX;
        distanceHorizontal = distanceToTarget * Math.sin(Math.toRadians(targetAngle-limelightX));
        return (targetAngle);
    }*/
    private double getAngleFromSkew(double skew) {
        if ( skew > -1.0*ANGLE_THRESHOLD || skew < -90.0+ANGLE_THRESHOLD) {
            return 0.0;
        }
        else if (skew < -45.0) {
            return 90.0+skew;
        }
        return skew;
    }

    /*private double checkAngleSign(double initialAngle) {
        if (checkAngleSignIterations < 5) {
            drivePower = 0.0;
            turnPower = 0.3;
            angleSign = 0;
            checkAngleSignIterations++;

        } else{
            turnPower = 0.0;
            drivePower = 0.0;
            if (getTargetAngle() < initialAngle) {
                angleSign = 1.0; //this might be backwards
            } else {
                angleSign = -1.0; 
            }
        }
        return angleSign;
    }*/

    public void cancelAutonomousAssist() {
        autonomousAssistInProgress = false;
    }

    public boolean autonomousAssistInProgress() {
        return autonomousAssistInProgress;
    }
    /*public void autonomousAssistInit() {
        checkAngleSignIterations = 0;
        angleSign = 0;
        cachedAngle = getTargetAngle();
    }*/

    public double[] seekTarget() {
        double[] ans = new double[1];
        if (limelightX < 0) ans[0] = 0.5;
        else ans[0] = -0.5;
        return ans;
    }

    //@return double array [power to go left/right, power to turn, power to move head, ready to shoot, distance to target]
    public double[] autonomousAssist() {
        double[] ans = new double[5];
        autonomousAssistInProgress = true;
        //ans[0] = distanceHorizontal; old
        if(limelightV == 0) {
            return seekTarget();
        }
        //Stops the aiming assist if we can't see the target.
        //TODO: Add seeking, and logic to figure out how to move if we can't see the target

        //[distance to go left/right, amount to move neck, distance from target, ready to shoot?]
        //sigmoid changes distance/angle to power input
        //ans[0] = -1.0*sigmoid(limelightX);
        //ans[1] = sigmoid(limelightY-LIMELIGHT_Y_OFFSET); // is this correct?
        //ans[2] = sigmoid(distanceToTarget,0.75,0.117);
        //ans[3] = 0.0; // 0 if not ready to shoot, 1 if ready, -1 if needs to move forward/backwards
    
        turnPower = sigmoid(targetAngle, 0.75, 0.2);
        if(Math.abs(targetAngle) == 0.0) {
            drivePower = -1.0*sigmoid(limelightX);
        }
        else drivePower = 0.0;
        

        //ans[0]= 0.0;
        //Uncomment once turning is working well.
        ans[0] = drivePower;
        ans[1] = turnPower;
        ans[2] = sigmoid(limelightY - LIMELIGHT_Y_OFFSET);
        ans[3] = 0;
        ans[4] = sigmoid(distanceToTarget,0.75,0.117);


        //two options: 1) change shooting power based on distance in shooter
        //2) change limelightY to be above the center and keep constant shooting power
        //i think #1 will be easier to implement, but we can try the other one too
        if (Math.abs(ans[0]) <= LIMELIGHT_X_Y_THRESHOLD && Math.abs(ans[2]) <= LIMELIGHT_X_Y_THRESHOLD && turnPower == 0) {
            //autonomousAssistInProgress = false;
            ans[3] = 1.0;
        }
        //handle these cases later
        /*if (distanceToTarget >= MAX_DISTANCE_FROM_TARGET || distanceToTarget <= MIN_DISTANCE_FROM_TARGET) {
            ans[3] = -1.0;
        }*/

        return ans;
    }

    //Updates and displays sensor values during teleop
    //@param actuatorPreviouslyMoving from turret subsystem
    public void teleopPeriodic(boolean actuatorPreviouslyMoving) {
        limelightV = table.getEntry("tv").getDouble(0);
        if (limelightV != 0.0) {
        limelightX = table.getEntry("tx").getDouble(0);
        limelightY = table.getEntry("ty").getDouble(0);
        limelightA = table.getEntry("ta").getDouble(0);
        limelightHOR = table.getEntry("tlong").getDouble(0);
        limelightVER = table.getEntry("tshort").getDouble(0);
        limelightS = table.getEntry("ts").getDouble(0);
        targetAngle = getAngleFromSkew(limelightS);
        }

        getLimelightDistance(actuatorPreviouslyMoving);
        displayValues();
        //getTargetAngle();
    }

}
