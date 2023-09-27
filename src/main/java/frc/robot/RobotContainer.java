package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.Auto.DriveForward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveMode;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

    private final RobotConfiguration robot = new RobotConfiguration(ChassisMode.B_TEAM, DriveMode.ARCADEDRIVE);

    private final DriveSubsystem drive;

    //private final Joystick leftStick = new Joystick(0);
    //private final Joystick rightStick = new Joystick(1);
    // private ArmSubsystem wrist = new ArmSubsystem();
    // private Intake intake = new Intake();

    private CommandXboxController gamepad = new CommandXboxController(0);

    public RobotContainer(){


        drive = new DriveSubsystem(ChassisMode.C_TEAM, DriveMode.ARCADEDRIVE);




        //fix 
        // if (robot.driveConfig() == DriveMode.TANKDRIVE) {
        //     drive.setDefaultCommand(new TankDrive(drive, 
        //         () -> gamepad.getLeftY(),
        //         () -> gamepad.getRightY()));
        // } else {
        //     drive.setDefaultCommand(new ArcadeDrive(drive, 
        //         () -> gamepad.getLeftY(), 
        //         () -> gamepad.getRightX()));
        // }

        drive.setDefaultCommand(new ArcadeDrive(drive,
                () -> 0.7*gamepad.getRightX(), 
                () -> -gamepad.getLeftY()));
        
        SendableChooser<String> autoChooser = new SendableChooser<String>();
        //private Command activeAutoCommand = null;
        //private String activeAutoName = null;
        configureButtonBindings();
   
    }

    private void configureButtonBindings() {
        // gamepad.x()
        //     .onTrue(new InstantCommand(intake::shoot))
        //     .onFalse(new InstantCommand(intake::stop));
        // gamepad.y()
        //     .onTrue(new InstantCommand(intake::spit))
        //     .onFalse(new InstantCommand(intake::stop));
        
        // gamepad.a()
        //     .onTrue(new InstantCommand(() -> swapDrive()));
    }

    // private void swapDrive() {
    //     drive.swapMode();
    //     if (drive.getMode() == DriveMode.TANKDRIVE) {
    //         drive.setDefaultCommand(new TankDrive(drive,
    //             () -> gamepad.getLeftY(),
    //             () -> gamepad.getRightY()));
    //     }
    //     else {
    //         drive.setDefaultCommand(new ArcadeDrive(drive, 
    //             () -> gamepad.getLeftY(), 
    //             () -> gamepad.getRightX()));
    //     }
    // }

    public Command getAutoCommand() {
        return new DriveForward(drive);
    }
}
