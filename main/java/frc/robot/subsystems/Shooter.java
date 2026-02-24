package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  private final SparkMax agitatorMotorOne;
  private final SparkMax agitatorMotorTwo;

  private double currentShooterPower = 0.0;

  public Shooter() {
    shooterIntakeMotor = new SparkMax(ShooterConstants.SHOOTER_INTAKE_MOTOR_ID, MotorType.kBrushless);
    middleShooterMotor = new SparkFlex(ShooterConstants.MIDDLE_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    agitatorMotorOne = new SparkMax(ShooterConstants.AGITATOR_MOTOR_ONE_ID, MotorType.kBrushless);
    agitatorMotorTwo = new SparkMax(ShooterConstants.AGITATOR_MOTOR_TWO_ID, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    intakeConfig.inverted(ShooterConstants.SHOOTER_INTAKE_INVERTED);
    shooterIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig middleConfig = new SparkFlexConfig();
    middleConfig.idleMode(IdleMode.kCoast);
    middleConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    middleConfig.inverted(ShooterConstants.MIDDLE_SHOOTER_INVERTED);
    middleShooterMotor.configure(middleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig agitatorOneConfig = new SparkMaxConfig();
    agitatorOneConfig.idleMode(IdleMode.kCoast);
    agitatorOneConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    agitatorOneConfig.inverted(ShooterConstants.AGITATOR_MOTOR_ONE_INVERTED);
    agitatorMotorOne.configure(agitatorOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig agitatorTwoConfig = new SparkMaxConfig();
    agitatorTwoConfig.idleMode(IdleMode.kCoast);
    agitatorTwoConfig.smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    agitatorTwoConfig.inverted(ShooterConstants.AGITATOR_MOTOR_TWO_INVERTED);
    agitatorMotorTwo.configure(agitatorTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stop() {
    shooterIntakeMotor.stopMotor();
    middleShooterMotor.stopMotor();
    agitatorMotorOne.stopMotor();
    agitatorMotorTwo.stopMotor();
    currentShooterPower = 0.0;
  }

  public void setShooterPower(double power) {
    currentShooterPower = MathUtil.clamp(power, -1.0, 1.0);
    middleShooterMotor.set(currentShooterPower);

    if (Math.abs(currentShooterPower) > 1e-3) {
      setAgitatorPower(ShooterConstants.AGITATOR_POWER);
    } else {
      setAgitatorPower(0.0);
    }
  }

  public void setShooterIntakePower(double power) {
    shooterIntakeMotor.set(MathUtil.clamp(power, -1.0, 1.0));
  }

  public void setAgitatorPower(double power) {
    if (!ShooterConstants.AGITATOR_ENABLED) {
      agitatorMotorOne.stopMotor();
      agitatorMotorTwo.stopMotor();
        return;
    }

    double clampedPower = MathUtil.clamp(power, -1.0, 1.0);
    agitatorMotorOne.set(clampedPower);
    agitatorMotorTwo.set(clampedPower);
  }

  public double getShooterPower() {
    return currentShooterPower;
  }

  public boolean atPower() {
    return Math.abs(getShooterPower() - ShooterConstants.SHOOTER_POWER_AT_10FT)
        <= ShooterConstants.SHOOTER_POWER_TOLERANCE;
  }

  public double getTargetPowerForDistanceInches(double distanceInches) {
    if (Double.isNaN(distanceInches) || Double.isInfinite(distanceInches)) {
      return ShooterConstants.SHOOTER_POWER_AT_10FT;
    }

    double distanceFeet = distanceInches / 12.0;
    double[] distancePoints = {5.0, 10.0, 15.0, 20.0};
    double[] powerPoints = {
        ShooterConstants.SHOOTER_POWER_AT_5FT,
        ShooterConstants.SHOOTER_POWER_AT_10FT,
        ShooterConstants.SHOOTER_POWER_AT_15FT,
        ShooterConstants.SHOOTER_POWER_AT_20FT};

    if (distanceFeet <= distancePoints[0]) {
      return MathUtil.clamp(powerPoints[0], -1.0, 1.0);
    }

    if (distanceFeet >= distancePoints[distancePoints.length - 1]) {
      return MathUtil.clamp(powerPoints[powerPoints.length - 1], -1.0, 1.0);
    }

    for (int i = 0; i < distancePoints.length - 1; i++) {
      double lowDistance = distancePoints[i];
      double highDistance = distancePoints[i + 1];
      if (distanceFeet >= lowDistance && distanceFeet <= highDistance) {
        double t = (distanceFeet - lowDistance) / (highDistance - lowDistance);
        return MathUtil.clamp(MathUtil.interpolate(powerPoints[i], powerPoints[i + 1], t), -1.0, 1.0);
      }
    }

    return MathUtil.clamp(ShooterConstants.SHOOTER_POWER_AT_10FT, -1.0, 1.0);
  }

  public Command runAgitatorPower(double power) {
    return startEnd(() -> setAgitatorPower(power), () -> setAgitatorPower(0.0));
  }

  public Command runShooterPower(double shooterPower) {
    return runShooterPower(() -> shooterPower, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterPower(DoubleSupplier shooterPowerSupplier) {
    return runShooterPower(shooterPowerSupplier, () -> ShooterConstants.SHOOTER_INTAKE_POWER);
  }

  public Command runShooterPower(DoubleSupplier shooterPowerSupplier, DoubleSupplier intakePowerSupplier) {
    final double[] startTimestamp = {-1.0};
    return runEnd(
        () -> {
          if (startTimestamp[0] < 0.0) {
            startTimestamp[0] = Timer.getFPGATimestamp();
          }

          setShooterPower(shooterPowerSupplier.getAsDouble());

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

  public Command runShooterPower() {
    return runShooterPower(ShooterConstants.SHOOTER_POWER_AT_10FT);
  }

  public Command spinUpForDistanceCommand(DoubleSupplier distanceInchesSupplier) {
    return runOnce(() -> setShooterPower(getTargetPowerForDistanceInches(distanceInchesSupplier.getAsDouble())));
  }

  public Command runShooterForSeconds(double seconds, DoubleSupplier distanceInchesSupplier) {
    return runShooterPower(() -> getTargetPowerForDistanceInches(distanceInchesSupplier.getAsDouble()))
        .withTimeout(seconds)
        .andThen(runOnce(this::stop));
  }

  public Command stopShooterCommand() {
    return runOnce(this::stop);
  }

  @Override
  public void close() {
    shooterIntakeMotor.close();
    middleShooterMotor.close();
    agitatorMotorOne.close();
    agitatorMotorTwo.close();
  }
}
