package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final CommandXboxController gamepad = new CommandXboxController(2);
    private ArmSubsystem wrist = new ArmSubsystem();
    private Intake intake = new Intake();

    //private XboxController gamepad = new XboxController(0);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;

    public RobotContainer() {
        if (drive.getMode() == DriveMode.TANKDRIVE){
            drive.setDefaultCommand(new TankDrive(drive,
                () -> gamepad.getLeftY(),
                () -> gamepad.getRightY())); }
        else{
            drive.setDefaultCommand(new ArcadeDrive(drive, 
                () -> gamepad.getLeftY(), 
                () -> gamepad.getRightX()));}
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        gamepad.x()
            .onTrue(new InstantCommand(intake::shoot))
            .onFalse(new InstantCommand(intake::stop));
        gamepad.y()
            .onTrue(new InstantCommand(intake::spit))
            .onFalse(new InstantCommand(intake::stop));
        
        gamepad.a()
            .onTrue(new InstantCommand(() -> swapDrive()));
    }

    private void swapDrive() {
        drive.swapMode();
        if (drive.getMode() == DriveMode.TANKDRIVE) {
            drive.setDefaultCommand(new TankDrive(drive,
                () -> gamepad.getLeftY(),
                () -> gamepad.getRightY()));
        }
        else {
            drive.setDefaultCommand(new ArcadeDrive(drive, 
                () -> gamepad.getLeftY(), 
                () -> gamepad.getRightX()));
        }
    }
}
