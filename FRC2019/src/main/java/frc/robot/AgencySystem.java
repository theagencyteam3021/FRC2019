package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AgencySystem {
    protected Boolean debug = false;
    protected String name;

    //NOTE: Not working in FRC2019 
    //Prints object to the console in debug mode
    //@param o something to print to the console
    protected void consoleDebug(Object o) {
        if (!debug)
            return;
        System.out.println(this.getClass().getName() + " - " + o.toString());
    }

    //Prints object to the console when in debug mode
    //@param key the header of the SmartDashboard value
    //@param o something to print to the console
    protected void shuffleDebug(String key, Object o) {
        if (!debug)
            return;
        SmartDashboard.putString(key, o.toString());
    }

    // We do not have a  RobotInit() because these objects should only be created in RobotInit
    public void simulationInit() {
    };

    public void disabledInit() {
    };

    public void autonomousInit() {
    };

    public void teleopInit() {
    };

    public void testInit() {
    };

    public void robotPeriodic() {
    };

    public void simulationPeriodic() {
    };

    public void disabledPeriodic() {
    };

    public void autonomousPeriodic() {
    };

    public void teleopPeriodic() {
    };

    public void testPeriodic() {
    };

}