package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import java.util.Set;

public class IntakePivot extends SubsystemBase implements AutoCloseable {
  private final SparkMax pivotMotor;
  private final SparkFlex wheelMotor;
  private final DutyCycleEncoder pivotEncoder;
  private final NetworkTableEntry pivotAngleDegreesEntry =
      NetworkTableInstance.getDefault().getTable("Elastic").getEntry("Intake Pivot Angle (deg)");

  public IntakePivot() {
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    wheelMotor = new SparkFlex(IntakeConstants.WHEEL_MOTOR_ID, MotorType.kBrushless);
    pivotEncoder = new DutyCycleEncoder(9);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.PIVOT_CURRENT_LIMIT_AMPS);
    pivotConfig.inverted(IntakeConstants.PIVOT_INVERTED);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig wheelConfig = new SparkFlexConfig();
    wheelConfig.idleMode(IdleMode.kBrake);
    wheelConfig.smartCurrentLimit(IntakeConstants.WHEEL_SPARKFLEX_CURRENT_LIMIT_AMPS);
    wheelConfig.inverted(IntakeConstants.WHEEL_INVERTED);
    wheelMotor.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setPivotPower(double power) {
    double requestedPower = power;
    double angle = getPivotAngleDegrees();
    if (requestedPower > 0.0 && angle <= IntakeConstants.PIVOT_MAX_INWARD_ANGLE) {
      requestedPower = 0.0;
    }
    if (requestedPower < 0.0 && angle >= IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE) {
      requestedPower = 0.0;
    }
    pivotMotor.set(requestedPower);
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public Command runPivotPower(double power) {
    return startEnd(() -> setPivotPower(power), this::stop);
  }

  private double wrapToSignedDegrees(double degrees) {
    return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  public double getPivotAngleDegrees() {
    double absoluteDegrees = pivotEncoder.get() * 360.0;
    // Flip sign convention so outward is positive and inward is negative.
    return -wrapToSignedDegrees(absoluteDegrees - IntakeConstants.PIVOT_ABSOLUTE_ENCODER_OFFSET_DEGREES);
  }

  private double shortestSignedErrorDegrees(double currentDegrees, double targetDegrees) {
    return wrapToSignedDegrees(targetDegrees - currentDegrees);
  }

  public Command runPivotClockwiseToAngle(double targetDegrees) {
    return runEnd(
        () -> {
          double current = getPivotAngleDegrees();
          double error = shortestSignedErrorDegrees(current, targetDegrees);
          if (Math.abs(error) <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES || error > 0.0) {
            stop();
          } else {
            setPivotPower(-Math.abs(IntakeConstants.PIVOT_POWER));
          }
        },
        this::stop);
  }

  public Command runPivotCounterClockwiseToAngle(double targetDegrees) {
    return runEnd(
        () -> {
          double current = getPivotAngleDegrees();
          double error = shortestSignedErrorDegrees(current, targetDegrees);
          if (Math.abs(error) <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES || error < 0.0) {
            stop();
          } else {
            setPivotPower(Math.abs(IntakeConstants.PIVOT_POWER));
          }
        },
        this::stop);
  }

  public boolean isNearAngle(double targetDegrees) {
    return Math.abs(shortestSignedErrorDegrees(getPivotAngleDegrees(), targetDegrees))
        <= IntakeConstants.PIVOT_ANGLE_TOLERANCE_DEGREES;
  }

  public Command moveToOutAngleCommand() {
    return runPivotCounterClockwiseToAngle(IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE)
        .until(() -> isNearAngle(IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE))
        .withTimeout(2.5)
        .andThen(runOnce(this::stop));
  }

  public Command moveToInAngleCommand() {
    return runPivotClockwiseToAngle(IntakeConstants.PIVOT_MAX_INWARD_ANGLE)
        .until(() -> isNearAngle(IntakeConstants.PIVOT_MAX_INWARD_ANGLE))
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

  public Command runPivotAgitation(double swingDegrees, double wheelPower) {
    return Commands.defer(
        () -> {
          double centerAngle = getPivotAngleDegrees();
          double inTarget = Math.max(IntakeConstants.PIVOT_MAX_INWARD_ANGLE, centerAngle - Math.abs(swingDegrees));
          double outTarget = Math.min(IntakeConstants.PIVOT_MAX_OUTWARD_ANGLE, centerAngle + Math.abs(swingDegrees));

          Command oscillate = Commands.sequence(
              runPivotClockwiseToAngle(inTarget).withTimeout(1.5),
              runPivotCounterClockwiseToAngle(outTarget).withTimeout(1.5))
              .repeatedly();

          Command spinWheels = Commands.run(() -> setWheelPower(wheelPower));

          return Commands.deadline(oscillate, spinWheels)
              .finallyDo(() -> {
                stopWheels();
                stop();
              });
        },
        Set.of(this));
  }

  @Override
  public void periodic() {
    if (pivotEncoder.isConnected()) {
      pivotAngleDegreesEntry.setDouble(getPivotAngleDegrees());
    }
  }

  @Override
  public void close() {
    pivotMotor.close();
    wheelMotor.close();
    pivotEncoder.close();
  }
}
