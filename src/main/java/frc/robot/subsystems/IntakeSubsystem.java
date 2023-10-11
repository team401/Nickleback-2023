package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;

public class IntakeSubsystem {
  private CANSparkMax leftIntakeMotor, rightIntakeMotor, topIntakeMotor;
  private double intakeGoalPower;
  private double intakeOff = 0;
  private double intakeOn = ArmConstants.intakeVoltage;

  public IntakeSubsystem() {
    leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);
    topIntakeMotor = new CANSparkMax(ArmConstants.topIntakeMotorID, MotorType.kBrushless);
    leftIntakeMotor.follow(rightIntakeMotor, true);
    leftIntakeMotor.setSmartCurrentLimit(80);
    rightIntakeMotor.setSmartCurrentLimit(80);
    topIntakeMotor.setSmartCurrentLimit(80);
  }
  public void intakeOn () {
    intakeGoalPower = intakeOn;
  }
  public void intakeOff () {
    intakeGoalPower = intakeOff;
  }
}
