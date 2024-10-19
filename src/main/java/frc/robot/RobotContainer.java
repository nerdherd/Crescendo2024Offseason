// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.deser.impl.BeanAsArrayBuilderDeserializer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.DriveForward;
import frc.robot.commands.autos.Taxi;
// import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.Climb;
// import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Reportable;
// import frc.robot.subsystems.ShooterPivot;
// import frc.robot.subsystems.ShooterRoller;
// import frc.robot.subsystems.SuperSystem;
// import frc.robot.subsystems.Tramp;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;
import frc.robot.util.NerdyMath;

public class RobotContainer implements Reportable {
  public Gyro imu = new PigeonV2(2);
  
  // public Indexer indexer = new Indexer();
  public IntakeRoller intakeRoller = new IntakeRoller();
  // public ShooterPivot shooterPivot = new ShooterPivot();
  // public ShooterRoller shooterRoller = new ShooterRoller();
  // public Tramp tramp = new Tramp();
  public Climb climb = new Climb();
  // public BeamBreakSensor intakeBeamBreak = new BeamBreakSensor(0); // TODO value is placeholder
  // public BeamBreakSensor trampBeamBreak = new BeamBreakSensor(1); // TODO value is placeholder
  // public BeamBreakSensor shooterBeamBreak = new BeamBreakSensor(2); // TODO value is placeholder

  // public SuperSystem superSystem = new SuperSystem(intakeRoller, indexer, climb, intakeBeamBreak, trampBeamBreak, shooterBeamBreak);

  public SwerveDrivetrain swerveDrive;
  public PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);
  
  private CommandPS4Controller commandDriverController;
  private PS4Controller driverController;
  private CommandPS4Controller commandOperatorController;
  private PS4Controller operatorController;

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  
  private SwerveJoystickCommand swerveJoystickCommand;
  private Command visionShotCommand;
  
  /**
   * The container for the robot. Contain
   * s subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    commandDriverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
    commandOperatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
    operatorController = commandOperatorController.getHID();
    driverController = commandDriverController.getHID();

    try {
      swerveDrive = new SwerveDrivetrain(imu);

    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }
    intakeRoller = new IntakeRoller();

    // LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightBackName);
    // LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightFrontName);

    // Configure the trigger bindings
    // Moved to teleop init
    

    DriverStation.reportWarning("Initalization complete", false);
    // NamedCommands.registerCommand("intakeBasic1", superSystem.intakeBasicHold());
    // NamedCommands.registerCommand("intakeBasic2", superSystem.stopIntaking());
    // NamedCommands.registerCommand("shootSequence2Far", superSystem.shootSequence2Far());
    // NamedCommands.registerCommand("shootSequence2", superSystem.shootSequence2());

  }

  static boolean isRedSide = false;

  public static void refreshAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRedSide = (alliance.get() == DriverStation.Alliance.Red);
    }
  }

  public static boolean IsRedSide() {
    return isRedSide;
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //     return alliance.get() == DriverStation.Alliance.Red;
    // }
    // return false;
  }

  public void initDefaultCommands_teleop() {
    // shooterPivot.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       double increment = Math.signum(
    //           NerdyMath.deadband(
    //             -operatorController.getLeftY(), //0.5 rev/second 
    //             -ControllerConstants.kDeadband, 
    //             ControllerConstants.kDeadband)
    //         ) * 0.1;
    //       SmartDashboard.putNumber("Increment", increment);
    //       shooterPivot.incrementPosition(increment);
    //          // (20 * x) degrees per second
    //         // If x = 0.1, then v = 2 degrees per second
    //     },
    //     shooterPivot
    //   ));

    swerveJoystickCommand = 
    new SwerveJoystickCommand(
      swerveDrive,
      () -> -commandDriverController.getLeftY(), // Horizontal translation
      commandDriverController::getLeftX, // Vertical Translation
      () -> {
        return commandDriverController.getRightX(); // Rotation
      },
      () -> false, // should be field oriented now on true
      () -> false, // tow supplier
      driverController::getR2Button, // Precision/"Sniper Button"
      () -> {
        if (driverController.getCircleButton() || driverController.getCrossButton() || driverController.getTriangleButton()) {
          return true;
        }
        return false;
      },
      () -> { // Turn To angle Direction | TODO WIP
        if (driverController.getCircleButton()) { // turn to amp
          if (!IsRedSide()){
            return 90.0;
          }
          return 270.0;
        }
        if (driverController.getCrossButton()) {
           return 180.0;
        }
        if(driverController.getTriangleButton()) {
          return 0.0;
        }
        return swerveDrive.getImu().getHeading();
      }
    );

    swerveDrive.setDefaultCommand(swerveJoystickCommand);
}

  public void initDefaultCommands_test() {}

  public void configureBindings_teleop() {

    commandDriverController.options().whileTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );
    
    // commandDriverController.L2().whileTrue(
    //   superSystem.shootSpeaker()
    //   );

    commandOperatorController.povUp().whileTrue(
      Commands.sequence(
        climb.setEnabledCommand(true),
        climb.setPositionStateTopCommand()

      )).onFalse(
        climb.setEnabledCommand(false)
      );

    commandOperatorController.povDown().whileTrue(
      Commands.sequence(
        climb.setEnabledCommand(true),
        climb.setPositionStateBottomCommand()

      )).onFalse(
        climb.setEnabledCommand(false)
      );
    commandOperatorController.L2().whileTrue(
      Commands.sequence(
        intakeRoller.setEnabledCommand(true),
        intakeRoller.intakeCommand()
        // Commands.waitUntil(() -> intakeBeamBreak.noteSensed())
      )).onFalse(
        intakeRoller.setEnabledCommand(false)
    );
    commandOperatorController.R2().whileTrue(
      Commands.sequence(
        intakeRoller.setEnabledCommand(true),
        intakeRoller.outtakeCommand()
        // Commands.waitUntil(() -> intakeBeamBreak.noteSensed())
      )).onFalse(
        intakeRoller.setEnabledCommand(false)
    );

    // commandOperatorController.triangle().whileTrue(
    //   Commands.sequence(
    //     shooterPivot.setEnabledCommand(true),
    //     shooterPivot.moveToNeutral()
    //   )).onFalse(
    //     shooterPivot.setEnabledCommand(false)


    // );

    // commandOperatorController.square().whileTrue(
    //   Commands.sequence(
    //     tramp.setEnabledCommand(true),
    //     tramp.setElevatorAmpCommand()
    //   )).onFalse(
    //     tramp.setEnabledCommand(false)
    //   );
    
    // commandOperatorController.cross().whileTrue(
    //   Commands.sequence(
    //     tramp.setEnabledCommand(true),
    //     tramp.setTrampShootCommand()
    //     //Commands.waitUntil(() -> trampBeamBreak.noteSensed())
    //   )).onFalse(
    //     tramp.setEnabledCommand(false)
    //   );
    
  }

  public void configureBindings_test() {}
  
  public void configureLEDTriggers() {}
  
  public void initShuffleboard() {
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    intakeRoller.initShuffleboard(loggingLevel);
    climb.initShuffleboard(loggingLevel);
    // indexer.initShuffleboard(loggingLevel);
    // shooterPivot.initShuffleboard(loggingLevel);
    // shooterRoller.initShuffleboard(loggingLevel);
    // tramp.initShuffleboard(loggingLevel);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
    // tab.addNumber("apriltag angle", () -> swerveDrive.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7));
  }

  public void initDefaultCommands() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  PathPlannerPath sourceauto = PathPlannerPath.fromPathFile("source taxi");
  public Command getAutonomousCommand() {
    Command currentAuto = new Taxi(swerveDrive, List.of(sourceauto));
    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }

  @Override
  public void reportToSmartDashboard(LOG_LEVEL priority) {}

  @Override
  public void initShuffleboard(LOG_LEVEL level) {
    ShuffleboardTab tab = Shuffleboard.getTab("RobotContainer");

    if (level == LOG_LEVEL.ALL) {
      tab.addBoolean("Driver R1", () -> driverController.getR1Button());
      tab.addBoolean("Driver R2 (Sniper Button)", () -> driverController.getR2Button());
      tab.addBoolean("Driver L1", () -> driverController.getL1Button());
      tab.addBoolean("Driver L2", () -> driverController.getL2Button());
      tab.addBoolean("Driver Share", () -> driverController.getShareButton());
      tab.addBoolean("Driver Options", () -> driverController.getOptionsButton());
      
      tab.addBoolean("Operator L1", () -> operatorController.getL1Button());
      tab.addBoolean("Operator L2", () -> operatorController.getL2Button());
      tab.addBoolean("Operator Square", () -> operatorController.getSquareButton());
      tab.addBoolean("Operator Cross", () -> operatorController.getCrossButton());
      tab.addBoolean("Operator Circle", () -> operatorController.getCircleButton());
      tab.addBoolean("Operator Triangle", () -> operatorController.getTriangleButton());
    }
  }
}
