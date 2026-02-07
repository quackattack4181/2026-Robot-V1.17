package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakePivot extends SubsystemBase implements AutoCloseable {
  private final SparkMax pivotMotor;

  public IntakePivot() {
    pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT_AMPS);
    pivotConfig.inverted(IntakeConstants.PIVOT_INVERTED);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  @Override
  public void close() {
    pivotMotor.close();
  }
}
