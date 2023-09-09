package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.NeoTankDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NeoDriveSubsystem;

public class RobotContainer {

    private NeoDriveSubsystem driveSubsystem;

    private Joystick gamepad;

    public RobotContainer() {
        gamepad = new Joystick(2);
        configureSubsystems();
    }

    private void configureSubsystems() {
        driveSubsystem = new NeoDriveSubsystem();
        driveSubsystem.setDefaultCommand(new NeoTankDrive(
            driveSubsystem,
            () -> -gamepad.getRawAxis(1),
            () -> -gamepad.getRawAxis(5)
        ));
    }

}