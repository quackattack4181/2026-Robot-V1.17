package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final SparkFlex leftClimberMotor;
  private final SparkFlex rightClimberMotor;
  private final DutyCycleEncoder absoluteEncoder;

  public Climber() {
    leftClimberMotor = new SparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    rightClimberMotor = new SparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(ClimberConstants.ABSOLUTE_ENCODER_CHANNEL);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(ClimberConstants.SPARKFLEX_CURRENT_LIMIT_AMPS);
    leftConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED);
    leftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.smartCurrentLimit(ClimberConstants.SPARKFLEX_CURRENT_LIMIT_AMPS);
    rightConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED);
    rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimberPower(double power) {
    double clampedPower = Math.max(-ClimberConstants.MAX_ALLOWED_POWER,
        Math.min(ClimberConstants.MAX_ALLOWED_POWER, power));

    double currentAngle = getClimberAngleDegrees();
    if (clampedPower > 0.0 && currentAngle >= ClimberConstants.MAX_FORWARD_ANGLE_DEGREES) {
      clampedPower = 0.0;
    }
    if (clampedPower < 0.0 && currentAngle <= ClimberConstants.MAX_BACKWARD_ANGLE_DEGREES) {
      clampedPower = 0.0;
    }

    leftClimberMotor.set(clampedPower);
    rightClimberMotor.set(clampedPower);
  }

  private double wrapToSignedDegrees(double degrees) {
    return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  public double getClimberAngleDegrees() {
    double absoluteDegrees = absoluteEncoder.get() * 360.0;
    return wrapToSignedDegrees(absoluteDegrees - ClimberConstants.CLIMBER_ABSOLUTE_ENCODER_OFFSET_DEGREES);
  }

  public void stop() {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  public Command runClimberPower(double power) {
    return startEnd(() -> setClimberPower(power), this::stop);
  }

  @Override
  public void close() {
    leftClimberMotor.close();
    rightClimberMotor.close();
    absoluteEncoder.close();
  }
}
