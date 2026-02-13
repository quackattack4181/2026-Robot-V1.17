package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkFlex topMotor;
  private final SparkFlex middleMotor;
  private final SparkFlex bottomMotor;

  private final RelativeEncoder topEncoder;
  private final SparkClosedLoopController topPid;
  private final SparkClosedLoopController middlePid;
  private final SparkClosedLoopController bottomPid;

  public Shooter() {
    topMotor = new SparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    middleMotor = new SparkFlex(ShooterConstants.MIDDLE_MOTOR_ID, MotorType.kBrushless);
    bottomMotor = new SparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast);
    topConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    topConfig.inverted(ShooterConstants.TOP_INVERTED);
    topConfig.closedLoop.pidf(
        ShooterConstants.KP,
        ShooterConstants.KI,
        ShooterConstants.KD,
        ShooterConstants.KF);
    topMotor.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig middleConfig = new SparkFlexConfig();
    middleConfig.idleMode(IdleMode.kCoast);
    middleConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    middleConfig.inverted(ShooterConstants.MIDDLE_INVERTED);
    middleConfig.closedLoop.pidf(
        ShooterConstants.KP,
        ShooterConstants.KI,
        ShooterConstants.KD,
        ShooterConstants.KF);
    middleMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.idleMode(IdleMode.kCoast);
    bottomConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    bottomConfig.inverted(ShooterConstants.BOTTOM_INVERTED);
    bottomConfig.closedLoop.pidf(
        ShooterConstants.KP,
        ShooterConstants.KI,
        ShooterConstants.KD,
        ShooterConstants.KF);
    bottomMotor.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topEncoder = topMotor.getEncoder();
    topPid = topMotor.getClosedLoopController();
    middlePid = middleMotor.getClosedLoopController();
    bottomPid = bottomMotor.getClosedLoopController();
  }

  public void stop() {
    topMotor.stopMotor();
    middleMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  public void setShooterRpm(double rpm) {
    topPid.setReference(rpm, ControlType.kVelocity);
  }

  public double getShooterRpm() {
    return topEncoder.getVelocity();
  }

  public boolean atSpeed() {
    return Math.abs(getShooterRpm() - ShooterConstants.SHOOTER_RPM)
        <= ShooterConstants.VELOCITY_TOLERANCE_RPM;
  }

  public double getTargetRpmForDistanceInches(double distanceInches) {
    if (Double.isNaN(distanceInches) || Double.isInfinite(distanceInches)) {
      return ShooterConstants.SHOOTER_RPM;
    }

    double distanceFeet = distanceInches / 12.0;
    double[] distancePoints = {5.0, 10.0, 15.0, 20.0, 25.0};
    double[] rpmPoints = {
        ShooterConstants.SHOOTER_RPM_AT_5FT,
        ShooterConstants.SHOOTER_RPM_AT_10FT,
        ShooterConstants.SHOOTER_RPM_AT_15FT,
        ShooterConstants.SHOOTER_RPM_AT_20FT,
        ShooterConstants.SHOOTER_RPM_AT_25FT};

    if (distanceFeet <= distancePoints[0]) {
      return rpmPoints[0];
    }

    if (distanceFeet >= distancePoints[distancePoints.length - 1]) {
      return rpmPoints[rpmPoints.length - 1];
    }

    for (int i = 0; i < distancePoints.length - 1; i++) {
      double lowDistance = distancePoints[i];
      double highDistance = distancePoints[i + 1];
      if (distanceFeet >= lowDistance && distanceFeet <= highDistance) {
        double t = (distanceFeet - lowDistance) / (highDistance - lowDistance);
        return MathUtil.interpolate(rpmPoints[i], rpmPoints[i + 1], t);
      }
    }

    return ShooterConstants.SHOOTER_RPM;
  }

  public Command runShooterRpm(double rpm) {
    final double[] startTimestamp = {-1.0};
    return runEnd(
        () -> {
          if (startTimestamp[0] < 0.0) {
            startTimestamp[0] = Timer.getFPGATimestamp();
          }

          middlePid.setReference(rpm, ControlType.kVelocity);
          bottomPid.setReference(rpm, ControlType.kVelocity);

          if (Timer.getFPGATimestamp() - startTimestamp[0]
              >= ShooterConstants.TOP_MOTOR_START_DELAY_SECONDS) {
            topPid.setReference(rpm, ControlType.kVelocity);
          } else {
            topMotor.stopMotor();
          }
        },
        () -> {
          startTimestamp[0] = -1.0;
          stop();
        });
  }

  public Command runShooterRpm(java.util.function.DoubleSupplier rpmSupplier) {
    final double[] startTimestamp = {-1.0};
    return runEnd(
        () -> {
          if (startTimestamp[0] < 0.0) {
            startTimestamp[0] = Timer.getFPGATimestamp();
          }

          double rpm = rpmSupplier.getAsDouble();
          middlePid.setReference(rpm, ControlType.kVelocity);
          bottomPid.setReference(rpm, ControlType.kVelocity);

          if (Timer.getFPGATimestamp() - startTimestamp[0]
              >= ShooterConstants.TOP_MOTOR_START_DELAY_SECONDS) {
            topPid.setReference(rpm, ControlType.kVelocity);
          } else {
            topMotor.stopMotor();
          }
        },
        () -> {
          startTimestamp[0] = -1.0;
          stop();
        });
  }

  public Command runShooterRpm() {
    return runShooterRpm(ShooterConstants.SHOOTER_RPM);
  }

  @Override
  public void close() {
    topMotor.close();
    middleMotor.close();
    bottomMotor.close();
  }
}
