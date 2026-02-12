package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase implements AutoCloseable {
  private final SparkMax pivotMotor;
  private final SparkMax wheelMotor;
  private final DutyCycleEncoder pivotEncoder;
  private double lastPrintTime;

  public IntakePivot() {
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    wheelMotor = new SparkMax(IntakeConstants.WHEEL_MOTOR_ID, MotorType.kBrushless);
    pivotEncoder = new DutyCycleEncoder(9);
    lastPrintTime = 0.0;

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT_AMPS);
    pivotConfig.inverted(IntakeConstants.PIVOT_INVERTED);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.idleMode(IdleMode.kBrake);
    wheelConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT_AMPS);
    wheelConfig.inverted(IntakeConstants.WHEEL_INVERTED);
    wheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setPivotPower(double power) {
    pivotMotor.set(power);
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public Command runPivotPower(double power) {
    return startEnd(() -> setPivotPower(power), this::stop);
  }

  public void setWheelPower(double power) {
    wheelMotor.set(power);
  }

  public void stopWheels() {
    wheelMotor.stopMotor();
  }

  public Command runWheelsPower(double power) {
    return startEnd(() -> setWheelPower(power), this::stopWheels);
  }

  public Command runWheelsPower() {
    return runWheelsPower(IntakeConstants.WHEEL_POWER);
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrintTime >= 0.25) {
      double dutyCycle = pivotEncoder.get();
      if (pivotEncoder.isConnected()) {
        double degrees = dutyCycle * 360.0;
        System.out.printf("Intake pivot angle: %.2f degrees%n", degrees);
      } else {
        System.out.printf("Intake pivot encoder not connected (duty cycle=%.3f)%n", dutyCycle);
      }
      lastPrintTime = now;
    }
  }

  @Override
  public void close() {
    pivotMotor.close();
    wheelMotor.close();
    pivotEncoder.close();
  }
}
