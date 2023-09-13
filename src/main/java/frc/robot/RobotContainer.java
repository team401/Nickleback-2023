package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveMode;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

    private final RobotConfiguration robot = new RobotConfiguration(ChassisMode.B_TEAM, DriveMode.ARCADEDRIVE);

    private final DriveSubsystem drive;

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);


    public RobotContainer(){

        drive = new DriveSubsystem(robot.chassisConfig(), robot.driveConfig());
        //fix 

        drive.setDefaultCommand(new TankDrive(drive, 
                () -> leftStick.getRawAxis(1),
                () -> rightStick.getRawAxis(0)));

    }


}
