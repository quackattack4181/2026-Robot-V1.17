package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkFlex middleShooterMotor;
  private final SparkMax shooterIntakeMotor;

  private final RelativeEncoder middleShooterEncoder;
  private final SparkClosedLoopController middleShooterPid;

  public Shooter() {
    shooterIntakeMotor = new SparkMax(ShooterConstants.SHOOTER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    middleShooterMotor = new SparkFlex(ShooterConstants.MIDDLE_SHOOTER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    intakeConfig.inverted(ShooterConstants.SHOOTER_INTAKE_INVERTED);
    shooterIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig middleConfig = new SparkFlexConfig();
    middleConfig.idleMode(IdleMode.kCoast);
    middleConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    middleConfig.inverted(ShooterConstants.MIDDLE_SHOOTER_INVERTED);
    middleConfig.closedLoop.pidf(
        ShooterConstants.SHOOTER_KP,
        ShooterConstants.SHOOTER_KI,
        ShooterConstants.SHOOTER_KD,
        ShooterConstants.SHOOTER_KF);
    middleShooterMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    middleShooterEncoder = middleShooterMotor.getEncoder();
    middleShooterPid = middleShooterMotor.getClosedLoopController();
  }

  public void stop() {
    shooterIntakeMotor.stopMotor();
    middleShooterMotor.stopMotor();
  }

  public void setShooterRpm(double rpm) {
    middleShooterPid.setReference(rpm, ControlType.kVelocity);
  }

  public void setShooterIntakePower(double power) {
    shooterIntakeMotor.set(MathUtil.clamp(power, -1.0, 1.0));
  }

  public double getShooterRpm() {
    return middleShooterEncoder.getVelocity();
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

  public Command runShooterRpm(double shooterRpm) {
    return runShooterRpm(() -> shooterRpm, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterRpm(DoubleSupplier shooterRpmSupplier) {
    return runShooterRpm(shooterRpmSupplier, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterRpm(DoubleSupplier shooterRpmSupplier, DoubleSupplier intakePowerSupplier) {
    final double[] startTimestamp = {-1.0};
    return runEnd(
        () -> {
          if (startTimestamp[0] < 0.0) {
            startTimestamp[0] = Timer.getFPGATimestamp();
          }

          setShooterRpm(shooterRpmSupplier.getAsDouble());

          if (Timer.getFPGATimestamp() - startTimestamp[0]
              >= ShooterConstants.SHOOTER_INTAKE_START_DELAY_SECONDS) {
            setShooterIntakePower(intakePowerSupplier.getAsDouble());
          } else {
            shooterIntakeMotor.stopMotor();
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
    shooterIntakeMotor.close();
    middleShooterMotor.close();
  }
}
