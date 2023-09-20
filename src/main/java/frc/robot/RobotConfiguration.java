package frc.robot;

import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveMode;

public class RobotConfiguration {

    public static DriveMode driveMode; 
    public ChassisMode chassisMode;
    public static boolean armEnabled;

    public RobotConfiguration(ChassisMode chassisConfig, DriveMode driveConfig) {
        this.driveMode = driveConfig; 
        this.chassisMode = chassisConfig;

        armEnabled = (chassisConfig == ChassisMode.B_TEAM);

    }

    public DriveMode driveConfig() {
        return driveMode;
    }

    public ChassisMode chassisConfig() {
        return chassisMode;
    }


    public boolean armEnabled() {
        return armEnabled;
    }

    
}
