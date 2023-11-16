package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.ArmSubsystemHardware;

public class Intake {
  private CANSparkMax leftMotor, rightMotor, topIntakeMotor;
  private double intakePower;
  private final double CURRENT_LIMIT = 25.0;
  private double topIntakeMotorPower = 0;

  public Intake() {
    leftMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);
    topIntakeMotor = new CANSparkMax(ArmConstants.topIntakeMotorID, MotorType.kBrushless);
    leftMotor.follow(rightMotor, true);
    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);
    topIntakeMotor.setSmartCurrentLimit(10);

    rightMotor.setIdleMode(IdleMode.kCoast);
    leftMotor.setIdleMode(IdleMode.kCoast);
    topIntakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void intake() {
    intakePower = ArmConstants.intakeVoltage;
    topIntakeMotorPower = ArmConstants.topIntakePower;
  }

  public void shootLow() {
    intakePower = ArmConstants.lowShootVoltage;
    topIntakeMotorPower = ArmConstants.topShootPower;
  }

  public void shootMid() {
    intakePower = ArmConstants.midShootVoltage;
    topIntakeMotorPower = ArmConstants.topShootPower;
  }

  public void shootHigh() {
    intakePower = ArmConstants.highShootVoltage;
    topIntakeMotorPower = ArmConstants.topShootPower;
  }

  public void spit() {
    intakePower = -ArmConstants.spitVoltage;
    topIntakeMotorPower = ArmConstants.topShootPower;
  }

  public void off() {
    intakePower = 0;
    topIntakeMotorPower = 0;
  }

  public void setIntakeMotorPower(double percent) {
    rightMotor.set(-percent);
    topIntakeMotor.set(topIntakeMotorPower);
  }

  public double getIntakeMotorAmps() {
    return rightMotor.getOutputCurrent();
  }

  private void checkIntakeAmps() {
    if (getIntakeMotorAmps() > CURRENT_LIMIT) {
      setIntakeMotorPower(0);
    }
  }

  public void run() {
    setIntakeMotorPower(intakePower);
    checkIntakeAmps();
  }
}
