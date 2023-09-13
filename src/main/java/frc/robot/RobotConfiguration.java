package frc.robot;

import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveMode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotConfiguration {

    private static DriveSubsystem driveSubsystem;
    private static boolean armEnabled;

    public RobotConfiguration(DriveMode driveConfig, ChassisMode chassisConfig) {

        driveSubsystem = new DriveSubsystem(chassisConfig, driveConfig);

        if (chassisConfig == ChassisMode.B_TEAM) {
            armEnabled = true;
        } else {
            armEnabled = false;
        }

    }

    public DriveSubsystem driveSubsystem() {
        return driveSubsystem;
    }
    
    public 
    
}
