// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final String AUTO_SELECTED_KEY = "Auto Selected";
  private static final String AUTO_OPTIONS_KEY = "Auto Options";
  private final NetworkTableEntry autoSelectedEntry;
  private final NetworkTableEntry autoOptionsEntry;
  private final Map<String, Command> autoOptions = new LinkedHashMap<>();
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/neo"));
  private final Shooter shooter = new Shooter();
  private final IntakePivot intakePivot = new IntakePivot();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverOne = new CommandXboxController(0);
  // private final CommandXboxController driverTwo = new CommandXboxController(1);



  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // <<<===== CHANGED from -
      () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // <<<===== CHANGED from -
      () -> -driverOne.getRightX(), // <<<===== CHANGED from -
      () -> -driverOne.getRightY()); // <<<===== CHANGED from -

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverOne.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //startUSBCamera();  // Enable USB Camera for dashboard

    // Configure the trigger bindings
    configureBindings();

    NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    autoSelectedEntry = elasticTable.getEntry(AUTO_SELECTED_KEY);
    autoOptionsEntry = elasticTable.getEntry(AUTO_OPTIONS_KEY);

    boolean isBlueAlliance = false;

    // boolean isBlueAlliance = DriverStation.getAlliance()
    //         .map(alliance -> alliance == DriverStation.Alliance.Blue)
    //         .orElse(false); // Default to Red if unknown

    // Auto-discover PathPlanner autos/paths from deploy and publish to Elastic.
    loadAutoOptions(isBlueAlliance);

    NamedCommands.registerCommand("AlignToTag", drivebase.aimAtLimelightTarget(VisionConstants.LIMELIGHT_NAME));
    
  }

  // USB Camera and it's settings
//  private void startUSBCamera() {
//        UsbCamera camera = CameraServer.startAutomaticCapture(0);
//        camera.setResolution(320, 240); // Adjust resolution if needed
//        camera.setFPS(25); // Adjust FPS for efficiency
//    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {

    //========================================
    //        Driver One Controls #1
    //========================================

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // Spin intake wheels while driverOne holds A
    driverOne.a().whileTrue(intakePivot.runWheelsPower(IntakeConstants.WHEEL_POWER));

    driverOne.leftTrigger(0.5)
             .whileTrue(drivebase.driveFieldOrientedWithLimelight(
                 () -> MathUtil.applyDeadband(-driverOne.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                 () -> MathUtil.applyDeadband(-driverOne.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                 VisionConstants.LIMELIGHT_NAME));

    driverOne.rightTrigger(0.5).whileTrue(
        shooter.runShooterRpm(
            () -> shooter.getTargetRpmForDistanceInches(
                drivebase.getLimelightTargetDistanceInches(VisionConstants.LIMELIGHT_NAME))));

    intakePivot.setDefaultCommand(intakePivot.run(intakePivot::stop));
    driverOne.povUp().whileTrue(
        intakePivot.runPivotCounterClockwiseToAngle(IntakeConstants.PIVOT_IN_ANGLE_DEGREES));
    driverOne.povDown().whileTrue(
        intakePivot.runPivotClockwiseToAngle(IntakeConstants.PIVOT_OUT_ANGLE_DEGREES));



    //========================================
    //        Driver Two Controls #2
    //========================================

  }

  private void loadAutoOptions(boolean isBlueAlliance) {
    autoOptions.clear();

    Path pathplannerDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
    Path autosDir = pathplannerDir.resolve("autos");

    if (Files.isDirectory(autosDir)) {
      try (var files = Files.list(autosDir).filter(path -> path.toString().endsWith(".auto"))
          .sorted(Comparator.comparing(path -> path.getFileName().toString()))) {
        files.forEach(path -> {
          String fileName = path.getFileName().toString();
          String autoName = fileName.substring(0, fileName.length() - 5);
          autoOptions.put(autoName, new PathPlannerAuto(autoName, isBlueAlliance));
        });
      } catch (IOException e) {
        System.err.println("Failed to read PathPlanner autos: " + e.getMessage());
      }
    }

    if (Files.isDirectory(pathplannerDir)) {
      try (var files = Files.list(pathplannerDir).filter(path -> path.toString().endsWith(".path"))
          .sorted(Comparator.comparing(path -> path.getFileName().toString()))) {
        files.forEach(path -> {
          String fileName = path.getFileName().toString();
          String pathName = fileName.substring(0, fileName.length() - 5);
          String optionName = "Path: " + pathName;
          try {
            autoOptions.putIfAbsent(
                optionName,
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)));
          } catch (Exception e) {
            System.err.println("Failed to load PathPlanner path '" + pathName + "': " + e.getMessage());
          }
        });
      } catch (IOException e) {
        System.err.println("Failed to read PathPlanner paths: " + e.getMessage());
      }
    }

    autoOptionsEntry.setStringArray(autoOptions.keySet().toArray(new String[0]));
    if (autoOptions.isEmpty()) {
      autoSelectedEntry.setString("");
      SmartDashboard.putData("Auto Chooser", autoChooser);
      return;
    }

    String defaultSelection = autoOptions.keySet().iterator().next();
    autoChooser.setDefaultOption(defaultSelection, defaultSelection);
    for (String option : autoOptions.keySet()) {
      if (!option.equals(defaultSelection)) {
        autoChooser.addOption(option, option);
      }
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
    autoSelectedEntry.setString(defaultSelection);
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (autoOptions.isEmpty()) {
      return null;
    }

    String fallbackAuto = autoOptions.keySet().iterator().next();
    String chooserSelection = autoChooser.getSelected();
    String selectedAuto = chooserSelection != null ? chooserSelection : autoSelectedEntry.getString(fallbackAuto);
    return autoOptions.getOrDefault(selectedAuto, autoOptions.get(fallbackAuto));
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

}
