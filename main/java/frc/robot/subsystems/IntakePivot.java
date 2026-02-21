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


  public double getPivotAngleDegrees() {
    return pivotEncoder.get() * 360.0;
  }

  private double clockwiseDistanceToTarget(double currentDegrees, double targetDegrees) {
    return (currentDegrees - targetDegrees + 360.0) % 360.0;
  }

  private double counterClockwiseDistanceToTarget(double currentDegrees, double targetDegrees) {
    return (targetDegrees - currentDegrees + 360.0) % 360.0;
  }

  private double maxClockwiseTravelWithinRange() {
    return clockwiseDistanceToTarget(
        IntakeConstants.PIVOT_IN_ANGLE_DEGREES,
        IntakeConstants.PIVOT_OUT_ANGLE_DEGREES);
  }

  private double maxCounterClockwiseTravelWithinRange() {
    return counterClockwiseDistanceToTarget(
        IntakeConstants.PIVOT_OUT_ANGLE_DEGREES,
        IntakeConstants.PIVOT_IN_ANGLE_DEGREES);
  }

  public Command runPivotClockwiseToAngle(double targetDegrees) {
    final double[] lastRemaining = {Double.POSITIVE_INFINITY};
    return runEnd(
        () -> {
          double current = getPivotAngleDegrees();
          double remaining = clockwiseDistanceToTarget(current, targetDegrees);
          boolean reachedTarget = remaining <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;
          boolean passedTarget = remaining > lastRemaining[0];
          boolean commandDirectionInvalid =
              remaining > maxClockwiseTravelWithinRange() + IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;

          if (reachedTarget || passedTarget || commandDirectionInvalid) {
            stop();
          } else {
            setPivotPower(-Math.abs(IntakeConstants.PIVOT_POWER));
            lastRemaining[0] = remaining;
          }
        },
        () -> {
          lastRemaining[0] = Double.POSITIVE_INFINITY;
          stop();
        });
  }

  public Command runPivotCounterClockwiseToAngle(double targetDegrees) {
    final double[] lastRemaining = {Double.POSITIVE_INFINITY};
    return runEnd(
        () -> {
          double current = getPivotAngleDegrees();
          double remaining = counterClockwiseDistanceToTarget(current, targetDegrees);
          boolean reachedTarget = remaining <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;
          boolean passedTarget = remaining > lastRemaining[0];
          boolean commandDirectionInvalid =
              remaining > maxCounterClockwiseTravelWithinRange() + IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;

          if (reachedTarget || passedTarget || commandDirectionInvalid) {
            stop();
          } else {
            setPivotPower(Math.abs(IntakeConstants.PIVOT_POWER));
            lastRemaining[0] = remaining;
          }
        },
        () -> {
          lastRemaining[0] = Double.POSITIVE_INFINITY;
          stop();
        });
  }


  public boolean isNearAngle(double targetDegrees) {
    double current = getPivotAngleDegrees();
    double delta = Math.abs(((current - targetDegrees + 540.0) % 360.0) - 180.0);
    return delta <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;
  }

  public Command moveToOutAngleCommand() {
    return runPivotClockwiseToAngle(IntakeConstants.PIVOT_OUT_ANGLE_DEGREES)
        .until(() -> isNearAngle(IntakeConstants.PIVOT_OUT_ANGLE_DEGREES))
        .withTimeout(2.5)
        .andThen(runOnce(this::stop));
  }

  public Command moveToInAngleCommand() {
    return runPivotCounterClockwiseToAngle(IntakeConstants.PIVOT_IN_ANGLE_DEGREES)
        .until(() -> isNearAngle(IntakeConstants.PIVOT_IN_ANGLE_DEGREES))
        .withTimeout(2.5)
        .andThen(runOnce(this::stop));
  }

  public Command stopWheelsCommand() {
    return runOnce(this::stopWheels);
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
        double degrees = getPivotAngleDegrees();
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
