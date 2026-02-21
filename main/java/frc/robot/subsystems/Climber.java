package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final SparkMax leftClimberMotor;
  private final SparkMax rightClimberMotor;

  public Climber() {
    leftClimberMotor = new SparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    rightClimberMotor = new SparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT_AMPS);
    leftConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED);
    leftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT_AMPS);
    rightConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED);
    rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimberPower(double power) {
    leftClimberMotor.set(power);
    rightClimberMotor.set(power);
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
  }
}
