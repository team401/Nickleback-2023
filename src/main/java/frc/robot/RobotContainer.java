package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmMove;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {

    private final DriveSubsystem drive = new DriveSubsystem();
    private ArmSubsystem arm = new ArmSubsystem();

    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

    private CommandXboxController gamepad = new CommandXboxController(2);

    private ArmMove armIntake = new ArmMove(arm, Mode.INTAKE);
    private ArmMove armShootHigh = new ArmMove(arm, Mode.SHOOT_HIGH);
    private ArmMove armShootMid = new ArmMove(arm, Mode.SHOOT_MID);
    private ArmMove armShootLow = new ArmMove(arm, Mode.SHOOT_LOW);
    private ArmMove armSpit = new ArmMove(arm, Mode.SPIT);

    SendableChooser<String> autoChooser = new SendableChooser<String>();

    public RobotContainer() {
        
        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftJoystick.getRawAxis(1),
            () -> -leftJoystick.getRawAxis(0),
            () -> -rightJoystick.getRawAxis(0),
            true
        ));
        configureButtonBindings();
        
        SmartDashboard.putNumber("Auto Drive Time", 5);
    }

    private void configureButtonBindings() {
        gamepad.a()
            .whileTrue(armIntake);
        gamepad.y()
            .whileTrue(armShootHigh);
        gamepad.b()
            .whileTrue(armShootMid);
        gamepad.x()
            .whileTrue(armShootLow);
        new Trigger(() -> gamepad.getLeftY() < -0.7)
            .whileTrue(armSpit);
        new JoystickButton(rightJoystick, 1)
            .whileTrue(new InstantCommand(
                () -> {drive.resetHeading();
                }));
            
    }



}
