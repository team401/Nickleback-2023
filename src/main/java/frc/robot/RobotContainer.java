package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Intake;
import frc.robot.commands.auto.Auto;
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
    private final ArmSubsystem arm;

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);
    private ArmSubsystem wrist = new ArmSubsystem();
    private Intake intake = new Intake();

    //private XboxController gamepad = new XboxController(0);

    public RobotContainer(){


        drive = new DriveSubsystem(robot.chassisConfig(), robot.driveConfig());
        arm = new ArmSubsystem();

        //fix 
        if (robot.driveConfig() == DriveMode.TANKDRIVE) {
            drive.setDefaultCommand(new TankDrive(drive, 
                () -> leftStick.getRawAxis(1),
                () -> rightStick.getRawAxis(0)));
        } else {
            drive.setDefaultCommand(new ArcadeDrive(drive, 
                () -> leftStick.getRawAxis(1), 
                () -> rightStick.getRawAxis(0)));
        }
        
        SendableChooser<String> autoChooser = new SendableChooser<String>();
        //private Command activeAutoCommand = null;
        //private String activeAutoName = null;
        configureButtonBindings();
   
    }

    private void configureButtonBindings() {
        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(new InstantCommand(intake::shoot))
            .onFalse(new InstantCommand(intake::stop));
        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(new InstantCommand(intake::spit))
            .onFalse(new InstantCommand(intake::stop));
            
    }

    public Command getAutonomousCommand() {
        double x = SmartDashboard.getNumber("auto drive time", 5);
        Command auto = new Auto(arm, drive, x);
        return auto;
    }

    

}
