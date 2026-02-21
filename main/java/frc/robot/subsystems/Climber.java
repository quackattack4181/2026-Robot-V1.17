package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase implements AutoCloseable {
  private final SparkMax leftClimberMotor;
  private final SparkMax rightClimberMotor;
  private final DutyCycleEncoder absoluteEncoder;
  private final NetworkTableEntry climberAngleDegreesEntry =
      NetworkTableInstance.getDefault().getTable("Elastic").getEntry("Climber Angle (deg)");

  public Climber() {
    leftClimberMotor = new SparkMax(ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    rightClimberMotor = new SparkMax(ClimberConstants.RIGHT_CLIMBER_MOTOR_ID, MotorType.kBrushless);
    absoluteEncoder = new DutyCycleEncoder(ClimberConstants.ABSOLUTE_ENCODER_CHANNEL);

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

  public double getClimberAngleDegrees() {
    return absoluteEncoder.get() * 360.0;
  }

  private double shortestSignedErrorDegrees(double currentDegrees, double targetDegrees) {
    return ((targetDegrees - currentDegrees + 540.0) % 360.0) - 180.0;
  }

  private double getPositionHoldPower(double targetDegrees) {
    double error = shortestSignedErrorDegrees(getClimberAngleDegrees(), targetDegrees);
    if (Math.abs(error) <= ClimberConstants.POSITION_TOLERANCE_DEGREES) {
      return 0.0;
    }

    return Math.max(-ClimberConstants.POSITION_MAX_POWER,
                    Math.min(ClimberConstants.POSITION_MAX_POWER,
                             error * ClimberConstants.POSITION_KP));
  }

  public Command holdAtAngle(double targetDegrees) {
    return run(() -> setClimberPower(getPositionHoldPower(targetDegrees)));
  }

  public Command moveToAngle(double targetDegrees) {
    return holdAtAngle(targetDegrees);
  }

  public Command moveToAndHold(double targetDegrees) {
    return holdAtAngle(targetDegrees);
  }

  public Command moveToDown() {
    return moveToAndHold(ClimberConstants.DOWN_ANGLE_DEGREES);
  }

  public Command moveToHome() {
    return moveToDown();
  }

  public Command moveToLevel1() {
    return moveToAndHold(ClimberConstants.LEVEL_ONE_ANGLE_DEGREES);
  }

  public Command moveToLevel2() {
    return moveToAndHold(ClimberConstants.LEVEL_TWO_ANGLE_DEGREES);
  }

  public Command lockAtCurrentPosition() {
    final double[] lockAngleDegrees = {Double.NaN};
    return runEnd(
        () -> {
          if (Double.isNaN(lockAngleDegrees[0])) {
            lockAngleDegrees[0] = getClimberAngleDegrees();
          }
          setClimberPower(getPositionHoldPower(lockAngleDegrees[0]));
        },
        () -> {
          lockAngleDegrees[0] = Double.NaN;
          stop();
        });
  }

  public Command runClimberPower(double power) {
    return startEnd(() -> setClimberPower(power), this::stop);
  }

  @Override
  public void periodic() {
    if (absoluteEncoder.isConnected()) {
      climberAngleDegreesEntry.setDouble(getClimberAngleDegrees());
    }
  }

  @Override
  public void close() {
    leftClimberMotor.close();
    rightClimberMotor.close();
    absoluteEncoder.close();
  }
}
