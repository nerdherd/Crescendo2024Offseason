// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.autos.DriveForward;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.ShooterRoller;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.Tramp;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.PigeonV2;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;
import frc.robot.util.NerdyMath;

public class RobotContainer implements Reportable {
  public Gyro imu = new PigeonV2(2);
  
  public Indexer indexer = new Indexer();
  public IntakeRoller intakeRoller = new IntakeRoller();
  public ShooterPivot shooterPivot = new ShooterPivot();
  public ShooterRoller shooterRoller = new ShooterRoller();
  public Tramp tramp = new Tramp();
  public Climb climb = new Climb();
  public BeamBreakSensor intakeBeamBreak = new BeamBreakSensor(0); // TODO value is placeholder
  public BeamBreakSensor trampBeamBreak = new BeamBreakSensor(1); // TODO value is placeholder
  public BeamBreakSensor shooterBeamBreak = new BeamBreakSensor(2); // TODO value is placeholder

  public SuperSystem superSystem = new SuperSystem(intakeRoller, indexer, shooterPivot, shooterRoller, tramp, climb, intakeBeamBreak, trampBeamBreak, shooterBeamBreak);

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

    LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightBackName);
    LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.kLimelightFrontName);

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
    shooterPivot.setDefaultCommand(
      new RunCommand(
        () -> {
          double increment = Math.signum(
              NerdyMath.deadband(
                -operatorController.getLeftY(), //0.5 rev/second 
                -ControllerConstants.kDeadband, 
                ControllerConstants.kDeadband)
            ) * 0.1;
          SmartDashboard.putNumber("Increment", increment);
          shooterPivot.incrementPosition(increment);
             // (20 * x) degrees per second
            // If x = 0.1, then v = 2 degrees per second
        },
        shooterPivot
      ));

    swerveJoystickCommand = 
    new SwerveJoystickCommand(
      swerveDrive,
      () -> -commandDriverController.getLeftY(), // Horizontal translation
      commandDriverController::getLeftX, // Vertical Translation
      // () -> 0.0, // debug
      () -> {
        // if (driverController.getL2Button()) {
        //   SmartDashboard.putBoolean("Turn to angle 2", true);
        //   double turnPower = apriltagCamera.getTurnToTagPower(swerveDrive, angleError, IsRedSide() ? 4 : 7, adjustmentCamera); 
        //   SmartDashboard.putNumber("Turn Power", turnPower);
        //   return turnPower;
        // }
        // SmartDashboard.putBoolean("Turn to angle 2", false);
        return commandDriverController.getRightX(); // Rotation
      },
      // driverController::getSquareButton, // Field oriented
      () -> false, // should be robot oriented now on true
      () -> false,
      // driverController::getCrossButton, // Towing
      // driverController::getR2Button, // Precision/"Sniper Button"
      () -> driverController.getR2Button(), // Precision mode (disabled)
      () -> {
        return (
          driverController.getR1Button() 
          || driverController.getL1Button() 
          || driverController.getL2Button() 
          || driverController.getCircleButton()
          || driverController.getTriangleButton()
          // || (
          //   driverController.getTouchpad() && superSystem.getIsPassing()
          // )
          // || driverController.getPSButton()
        ); // Turn to angle
      }, 
      // () -> false, // Turn to angle (disabled)
      () -> { // Turn To angle Direction | TODO WIP
        // if (
        // (driverController.getTouchpad() && superSystem.getIsPassing())
        // || 
        // driverController.getTriangleButton()) { // turn shooter to speaker
        //   return 0.0;
        // }
        // if (driverController.getSquareButton()) { // turn shooter to shoot over stage
        //   if (!IsRedSide()) {
        //     return 315.0;
        //   } else {
        //     return 45.0;
        //   }
        // }
        if (driverController.getCircleButton()) { // turn to amp
          if (!IsRedSide()){
            return 90.0;
          }
          return 270.0;
        }
        // if (driverController.getCrossButton()) {
        //   return 180.0;
        // }
        if (driverController.getL2Button()) {
          return swerveDrive.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7); // 4 if red side, 7 if blue | TODO, update?
        }
        if (driverController.getL1Button()) {
          return 0.0;
        }
        if (driverController.getR1Button()) {
          return 0.0;
        }
        // if (driverController.getPSButton()) { // Turn to shuffleboard angle
        //   return SmartDashboard.getNumber("Test Desired Angle", 0);
        // }
        return 0.0;
      }

    );
    swerveDrive.setDefaultCommand(swerveJoystickCommand);
}

  public void initDefaultCommands_test() {}

  // Trigger armTrigger = new Trigger(
  //   () -> shooterPivot.atTargetPositionAccurate()
  //       && shooterPivot.getTargetPositionDegrees() > ShooterConstants.kFullStowPosition.get()
  //       && shooterRoller.getVelocityLeft() > (shooterRoller.getTargetVelocityLeft() * 0.6)
  //       && shooterRoller.getTargetVelocityLeft() > 0
  //       && shooterRoller.getVelocityRight() > (shooterRoller.getTargetVelocityRight() * 0.6)
  //       && shooterRoller.getTargetVelocityRight() > 0
  // );

  public void configureBindings_teleop() {
    // commandDriverController.share() is unbound
    commandDriverController.options().whileTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );
    // commandDriverController.touchpad().whileTrue(superSystem.shoot())
    //   .whileFalse(superSystem.stow());
    
    commandDriverController.L2().whileTrue(Commands.sequence( // copypasted from old code
      Commands.race(
        superSystem.shootSpeakerAutoAim(),
        Commands.sequence(
          Commands.race(
            // Commands.waitUntil(() -> 
            //   // aimTrigger.getAsBoolean() 
            //   // && 
            //   armTrigger.getAsBoolean()
            // ),
            Commands.waitSeconds(1.5)
          ),
          Commands.waitSeconds(0.1),
          superSystem.indexer.setEnabledCommand(true),
          superSystem.indexer.indexCommand(),
          Commands.either(
            Commands.none(),
            Commands.waitSeconds(0.5),
            () -> shooterBeamBreak.noteSensed()
          ),
          Commands.waitUntil(() -> !shooterBeamBreak.noteSensed())
        )
      ),
      superSystem.stow()
    )).whileFalse(superSystem.stow());
    // commandDriverController.R2() is in SwerveJoysitckCommand
    commandDriverController.L1().whileTrue(superSystem.shootSpeaker());
    commandDriverController.L2().whileTrue(superSystem.shootAmp());
    commandDriverController.share().whileTrue(
      Commands.runOnce(() -> swerveDrive.zeroGyroAndPoseAngle())
    );
    // commandDriverController.options() is unbound
    commandDriverController.touchpad().whileTrue(superSystem.shoot())
      .whileFalse(superSystem.stow());
    // Driver lleft joystick controls SwerveDrive movement
    // Driver lleft joystick controls SwerveDrive turning
    commandOperatorController.R2().whileTrue(superSystem.climbSequenceHoldCommand()).onFalse(superSystem.climbSequenceRealeaseCommand());
    commandOperatorController.L2().whileTrue(superSystem.intakeUntilSensed());
    //commandOperatorController.R2().whileTrue(shooterRoller.shootSpeaker());
    commandOperatorController.L1().whileTrue(
      Commands.sequence(
        intakeRoller.setEnabledCommand(true),
        intakeRoller.setVelocityCommand(-8)
      )).onFalse(intakeRoller.stopCommand());
    commandOperatorController.R1().whileTrue(
      Commands.sequence(
        intakeRoller.setEnabledCommand(true),
        intakeRoller.setVelocityCommand(8)
      )).onFalse(intakeRoller.stopCommand());
      commandOperatorController.triangle().whileTrue(indexer.indexToElevatorCommand());
      commandOperatorController.square().whileTrue(tramp.setElevatorTrapCommand());
      commandOperatorController.circle().whileTrue(tramp.setElevatorTrapCommand());
      commandOperatorController.cross().whileTrue(tramp.setTrampShootCommand());
      // commandOperatorController.share() is unbound
      // commandOperatorController.share() is unbound
      commandOperatorController.touchpad().whileTrue(climb.setPositionStateTopCommand());
      // Operator left joystick manually controlls ShooterPivot
    // commandDriverController.R2() is in SwerveJoystickCommand
    // commandDriverController.L1().whileTrue(superSystem.shootSpeaker()); is in SwerveJoystickCommand
    // commandDriverController.L2().whileTrue(superSystem.shootAmp()); is in SwerveJoystickCommand
    // Driver left joystick controls SwerveDrive movement
    // Driver right joystick controls SwerveDrive turning

    // commandOperatorController.R2().whileTrue(shooterRoller.shootSpeaker()); -> is unbound
    // commandOperatorController.L1().whileTrue( -> is unbound
    //   Commands.sequence(
    //     intakeRoller.setEnabledCommand(true),
    //     intakeRoller.setVelocityCommand(-8)
    //   )).onFalse(intakeRoller.stopCommand());
    // commandOperatorController.R1().whileTrue(
    //   Commands.sequence(
    //     intakeRoller.setEnabledCommand(true),
    //     intakeRoller.setVelocityCommand(8)
    //   )).onFalse(intakeRoller.stopCommand());
    // commandOperatorController.triangle().whileTrue(indexer.indexToElevatorCommand()); -> is unbound
    commandOperatorController.square().whileTrue(tramp.setElevatorTrapCommand());
    commandOperatorController.circle().whileTrue(tramp.setElevatorAmpCommand());
    commandOperatorController.cross().whileTrue(tramp.setTrampShootCommand());
    // commandOperatorController.share() is unbound
    // commandOperatorController.share() is unbound
    // commandOperatorController.touchpad().whileTrue(climb.setPositionStateTopCommand()); -> is unbound
    // Operator left joystick manually controlls ShooterPivot -> is unbound
  }

  public void configureBindings_test() {}
  
  public void configureLEDTriggers() {}
  
  public void initShuffleboard() {
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    intakeRoller.initShuffleboard(loggingLevel);

    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
    tab.addNumber("apriltag angle", () -> swerveDrive.getTurnToSpecificTagAngle(IsRedSide() ? 4 : 7));
  }

  public void initDefaultCommands() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = new DriveForward(swerveDrive);

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
