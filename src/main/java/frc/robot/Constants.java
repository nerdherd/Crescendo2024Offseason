// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.preferences.PrefBool;
import frc.robot.util.preferences.PrefDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // COMMENT ROBOT IDS INSTEAD OF DELETING
public final class Constants {

  public static class DriveConstants {
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;
    public static final double kErrorBound = 0;
  }

  public static class ControllerConstants {
    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 21.428; // 150 : 7 : 1 MK4i
    public static final double kDriveDistanceLoss = 1; // from measuring IRL
    public static final double kMetersPerRevolution = kWheelDiameterMeters * Math.PI * kDriveDistanceLoss;
    public static final double kDriveTicksToMeters = (1 / 2048.0) * kMetersPerRevolution; 
    public static final double kAbsoluteTurningTicksToRad = (1.0 / 4096.0) * 2 * Math.PI;
    public static final double kIntegratedTurningTicksToRad = (1.0 / 2048.0) * 2 * Math.PI;
    public static final double kDriveTicksPer100MsToMetersPerSec = kDriveTicksToMeters * 10;
    public static final double kAbsoluteTurningTicksPer100MsToRadPerSec = kAbsoluteTurningTicksToRad * 10;
    public static final double kIntegratedTurningTicksPer100MsToRadPerSec = kIntegratedTurningTicksToRad * 10;

    public static final double kDriveMotorDeadband = 0.02;
    public static final double kTurnMotorDeadband = 0.001;

    public static final PrefDouble kPTurning = new PrefDouble("kPTurning",0.55); // 0.55 abruticus
    public static final PrefDouble kITurning = new PrefDouble("kITurning",0); //0 abructicus
    public static final PrefDouble kDTurning = new PrefDouble("kDTurning",0.02); //0.02 abructicus
    public static final PrefDouble kFTurning = new PrefDouble("kFTurning",0.015); //0.015 abructicus

    public static final PrefDouble kPDrive = new PrefDouble("kPDrive",0.13); // 0.13 abruticus
    public static final PrefDouble kIDrive = new PrefDouble("kIDrive",0); //0 abructicus
    public static final PrefDouble kDDrive = new PrefDouble("kDDrive",0); //0 abructicus
    public static final PrefDouble kVDrive = new PrefDouble("kVDrive",0.0469); //0.0469 abructicus

    public static final String kCANivoreName = "CANivore";

    public static final double driverMotorCurrentLimit = 40.0;
    public static final double driverMotorCurrentThreshold = 30.0;
    public static final double turnMotorCurrentLimit = 30.0;
    public static final double turnMotorCurrentThreshold = 20.0;

  } 

  public static final class SwerveDriveConstants {

    public static final PrefDouble kPThetaTeleop = new PrefDouble("kP Theta Teleop", 4); //4 abruticus
    public static final PrefDouble kIThetaTeleop = new PrefDouble("kI Theta Teleop", 0); //0 abruticus
    public static final PrefDouble kDThetaTeleop = new PrefDouble("kD Theta Teleop", 0.1); //0.1 abruticus

    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21); 
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(21); 

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
    public static final int kFRDriveID = 11; 
    public static final int kFLDriveID = 21;
    public static final int kBLDriveID = 31;
    public static final int kBRDriveID = 41;

    public static final int kFRTurningID = 12;
    public static final int kFLTurningID = 22;
    public static final int kBLTurningID = 32;
    public static final int kBRTurningID = 42;

    public static final boolean kFRTurningReversed = true;
    public static final boolean kFLTurningReversed = true; 
    public static final boolean kBLTurningReversed = true; 
    public static final boolean kBRTurningReversed = true; 

    public static final boolean kFRDriveReversed = false;
    public static final boolean kFLDriveReversed = false;     
    public static final boolean kBLDriveReversed = false;      
    public static final boolean kBRDriveReversed = false;

    public static final class CANCoderConstants {
      public static final int kFRCANCoderID = 14;
      public static final int kFLCANCoderID = 24;
      public static final int kBLCANCoderID = 34;
      public static final int kBRCANCoderID = 44;

      public static final boolean kFRCANCoderReversed = false;    
      public static final boolean kFLCANCoderReversed = false;      
      public static final boolean kBLCANCoderReversed = false;       
      public static final boolean kBRCANCoderReversed = false; 
    }

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;    
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleMaxAcceleration = 5;
    // THIS CONSTANT HAS TO BE NEGATIVE OTHERWISE THE ROBOT WILL CRASH
    //TODO: Change deceleration with driver feedback, only in small increments (<= -2 is dangerous)
    public static final double kTeleMaxDeceleration = -5; // Russell says he likes 2.5 from sims, but keep at 3 until tested on real robot 

    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
      kPhysicalMaxAngularSpeedRadiansPerSecond * 0.75;
    public static final double kTurnToAngleMaxAngularSpeedRadiansPerSecond 
      = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTurnToBigAngleMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double kMinimumMotorOutput = 0.05; // Minimum percent output on the falcons
    
    public static final double kDriveAlpha = 0.11765;
    public static final double kDriveOneMinusAlpha = 0.88235;

    public static final SwerveModuleState[] towModuleStates = 
    new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(135))
    };

    public static final double kGravityMPS = 9.80665; 
  }

  public static final class SwerveAutoConstants {
    public static final double kPTurnToAngle = 6;
    public static final double kITurnToAngle = 0;
    public static final double kDTurnToAngle = 0.2;
    public static final double kTurnToAnglePositionToleranceAngle = 5;
    public static final double kTurnToAngleVelocityToleranceAnglesPerSec = 2;
    public static final double kTurnToAngleFeedForwardDegreesPerSecond = 6;
    
  }

  public static final class PathPlannerConstants {
    public static final double kPPMaxVelocity = 3; 
    public static final double kPPMaxAcceleration = 3;
    public static final double kPPMaxAngularVelocity = Math.PI * 2;
    public static final double kPPMaxAngularAcceleration = Math.PI * 2;
    public static final PathConstraints kPPPathConstraints = new PathConstraints(kPPMaxVelocity, kPPMaxAcceleration, 
                                                                                kPPMaxAngularVelocity, kPPMaxAngularAcceleration);

    public static final double kPP_P = new PrefDouble("PP_kP", 0.25).get();
    public static final double kPP_I = new PrefDouble("PP_kI", 0.0).get();
    public static final double kPP_D = new PrefDouble("PP_kD", 0.0).get();
    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = new PrefDouble("PP_kThetaP", 0.25).get();
    public static final double kPP_ThetaI = new PrefDouble("PP_kThetaI", 0).get();
    public static final double kPP_ThetaD = new PrefDouble("PP_kThetaD", 0).get();
    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);

    public static final boolean kUseAllianceColor = true;
  }
      
  public static final class SuperStructureConstants {
    public static final String kCANivoreBusName = "rio";
  }

  public static final class ShooterConstants {
    public static final int kTopMotorID = 17; //17 abruticus
    public static final int kBottomMotorID = 18; //18 abruticus
    public static final int kIntakePower = 0; //0 abruticus
    public static final int kOuttakePower = 0; //0 abruticus
    public static final PrefDouble kPTopMotor = new PrefDouble("P Top Motor", 0);
    public static final PrefDouble kITopMotor = new PrefDouble("I Top Motor", 0);
    public static final PrefDouble kDTopMotor = new PrefDouble("D Top Motor", 0);
    public static final PrefDouble kVTopMotor = new PrefDouble("V Top Motor", 0);

    public static final PrefDouble kPBottomMotor = new PrefDouble("P Bottom Motor", 0);
    public static final PrefDouble kIBottomMotor = new PrefDouble("I Bottom Motor", 0);
    public static final PrefDouble kDBottomMotor = new PrefDouble("D Bottom Motor", 0);
    public static final PrefDouble kVBottomMotor = new PrefDouble("V Bottom Motor", 0);

    public static final int kLeftMotorID = 51; //51 abruticus
    public static final int kRightMotorID = 52; //52 abruticus
    public static final int kShooterPigeonID = 9; //9 abrtuicus
    public static final int kLeftPivotMotorID = 53; //53 abruticus
    public static final int kRightPivotMotorID = 54; //54 abruticus
    public static final int kThroughBorePort = 3; //3 abruticus
 
    // In revolutions!
    public static final double kShooterNeutralDeadband = 0.01;
 
    // ************************************** SHOOTER CONSTANTS *************************************** //
 
    public static final double kShooterMaxVelocityRPS =  100;
    public static final double kShooterMinVelocityRPS = -100;
 
    public static final PrefDouble kLeftOuttakeHigh = new PrefDouble("Top Shooter Outtake High", 70); // Was 50    3/3/24 Code Orange
    public static final PrefDouble kLeftOuttakeLow  = new PrefDouble("Top Shooter Outtake Low",  20);
    public static final PrefDouble kTopOuttakeLowAutostart  = new PrefDouble("Top Shooter Outtake Low", 50); // to save spain time
    public static final PrefDouble kTopOuttakeHighAutostart = new PrefDouble("Top Shooter Outtake High Auto", 70); // to save spain time
    public static final PrefDouble kLeftOuttakeMidAutostart = new PrefDouble("Top Shooter Outtake Auto",      60);
    public static final PrefDouble kLeftOuttakeAuto1 = new PrefDouble("Top Shooter Outtake Auto 1",  20);
    public static final PrefDouble kLeftOuttakeAuto2 = new PrefDouble("Top Shooter Outtake Auto 2", 100);
    public static final PrefDouble kTopOuttakeAuto3  = new PrefDouble("Top Shooter Outtake Auto 3", 100);
    public static final PrefDouble kLeftOuttakeAmp   = new PrefDouble("Top Shooter Outtake Amp", 10);
    public static final PrefDouble kLeftIntake       = new PrefDouble("Top Shooter Intake", -10);
 
    public static final PrefDouble kRightOuttakeHigh = new PrefDouble("Bottom Shooter Outtake High", 80);
    public static final PrefDouble kRightOuttakeLow  = new PrefDouble("Bottom Shooter Outtake Low",  20);
    public static final PrefDouble kRightOuttakeAuto1  = new PrefDouble("Bottom Shooter Outtake Auto 1", 20);
    public static final PrefDouble kRightOuttakeAuto2  = new PrefDouble("Bottom Shooter Outtake Auto 2", 60);
    public static final PrefDouble kBottomOuttakeAuto3 = new PrefDouble("Bottom Shooter Outtake Auto 3", 60);
    public static final PrefDouble kRightOuttakeAmp = new PrefDouble("Bottom Shooter Outtake Amp", 10);
    public static final PrefDouble kRightIntake     = new PrefDouble("Bottom Shooter Intake", -10);
 
    public static final PrefDouble kPLeftMotor = new PrefDouble("kP Left Shooter", 0.2);
    public static final PrefDouble kILeftMotor = new PrefDouble("kI Left Shooter", 0.0004);
    public static final PrefDouble kDLeftMotor = new PrefDouble("kD Left Shooter", 0);
    public static final PrefDouble kVLeftMotor = new PrefDouble("kV Left Shooter", 0.15);
 
    public static final PrefDouble kPRightMotor = new PrefDouble("kP Right Shooter", 0.2);
    public static final PrefDouble kIRightMotor = new PrefDouble("kI Right Shooter", 0.0004);
    public static final PrefDouble kDRightMotor = new PrefDouble("kD Right Shooter", 0);
    public static final PrefDouble kVRightMotor = new PrefDouble("kV Right Shooter", 0.15);
 
    // ************************************** PIVOT CONSTANTS *************************************** //
 
    public static final double kPivotGearRatio = 225;
    public static final boolean kPivotAbsoluteEncoderInverted = true;
 
    public static final PrefBool fullDisableShooter = new PrefBool("Full Disable Shooter Pivot", false);
 
    public static final double kPivotMaxPos = 108;
    public static final double kPivotMinPos = -72;
 
    public static final PrefDouble kSpeakerPosition     = new PrefDouble("Pivot Speaker Position", -52.5);
    public static final PrefDouble kSpeakerPositionAuto = new PrefDouble("Pivot Speaker Position Auto", -46);
    public static final PrefDouble kSpeakerPositionAutoStart  = new PrefDouble("Pivot Speaker Position Auto", -40); 
    public static final PrefDouble kSpeakerPositionAutoStart2 = new PrefDouble("Pivot Speaker Position Auto", -27); 
    public static final PrefDouble kSpeakerPosition2 = new PrefDouble("Pivot Speaker Position 2", -31);
    public static final PrefDouble kSpeakerPosition3 = new PrefDouble("Pivot Speaker Position 3", -29);
    public static final PrefDouble kNeutralPosition  = new PrefDouble("Pivot Neutral Position", 36);
    public static final PrefDouble kAmpPosition      = new PrefDouble("Pivot Amp Position"    , 41.6);
    public static final PrefDouble kHandoffPosition  = new PrefDouble("Pivot Handoff Position",  -45);
    public static final PrefDouble kEjectPosition    = new PrefDouble("Pivot Eject Position",    -30);
    public static final PrefDouble kHandoffPosition2 = new PrefDouble("Pivot Handoff Position2", -37);  
   
    public static final PrefDouble k4PieceHandoffPosition1 = new PrefDouble("4 Piece Handoff Position 1", -37);
    public static final PrefDouble k4PieceHandoffPosition2 = new PrefDouble("4 Piece Handoff Position 2", -37);
    public static final PrefDouble k4PieceHandoffPosition3 = new PrefDouble("4 Piece Handoff Position 3", -37);  
    public static final PrefDouble k6PieceHandoffPosition  = new PrefDouble("6 Piece Handoff Position", -43);
 
    public static final PrefDouble kFullStowPosition  = new PrefDouble("Pivot Full Stow Position", -60);
    public static final PrefDouble kPrepClimbPosition = new PrefDouble("Pivot Prep Climb Position", 0);
 
    public static final PrefDouble kPPivotMotor = new PrefDouble("kP Shooter Pivot", 60); //Old values from Abruticus
    public static final PrefDouble kIPivotMotor = new PrefDouble("kI Shooter Pivot",  0); //Old values from Abruticus
    public static final PrefDouble kDPivotMotor = new PrefDouble("kD Shooter Pivot",  0); //Old values from Abruticus
    public static final PrefDouble kVPivotMotor = new PrefDouble("kV Shooter Pivot", 25); //Old values from Abruticus
    public static final PrefDouble kSPivotMotor = new PrefDouble("kS Shooter Pivot",  0.26); //Old values from Abruticus
    public static final PrefDouble kAPivotMotor = new PrefDouble("kA Shooter Pivot",  0.5); //Old values from Abruticus
    public static final PrefDouble kGPivotMotor = new PrefDouble("kG Shooter Pivot",  0.44); //Old values from Abruticus
 
    public static final PrefDouble kCruiseAcceleration = new PrefDouble("Shooter Pivot Cruise Acceleration", 1.7);
    public static final PrefDouble kCruiseVelocity     = new PrefDouble("Shooter Pivot Cruise Velocity", 0.425);
 
    // in degrees
    public static final PrefDouble kPivotDeadband = new PrefDouble("Shooter Pivot Deadband", 14.4);
    public static final PrefDouble kPivotDeadbandClose = new PrefDouble("Shooter Pivot Deadband Close", 2);
 
    // in degrees
    public static final PrefDouble kPivotOffset = new PrefDouble("Shooter Pivot Offset", 0); //Old values from Abruticus 248.4
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorID = 56; //change later

    public static final double kIntakeNeutralDeadband = 0.01;

    // ************************************** INTAKE CONSTANTS *************************************** //

    public static final PrefDouble kIntakeVelocity = new PrefDouble("Intake Velocity", -20);
    public static final PrefDouble kOutakeVelocity = new PrefDouble("Outake Velocity", 40);
    public static final PrefDouble kAutoIntakeVelocity = new PrefDouble("Intake Velocity Auto", -20);
    public static final double kIntakeMaxVelocity =  20;
    public static final double kIntakeMinVelocity = -20;

    public static final PrefDouble kPIntakeMotor = new PrefDouble("kP Intake Roller", 1); //Old values from Abruticus
    public static final PrefDouble kIIntakeMotor = new PrefDouble("kI Intake Roller", 0); //Old values from Abruticus
    public static final PrefDouble kDIntakeMotor = new PrefDouble("kD Intake Roller", 0.003); //Old values from Abruticus
    public static final PrefDouble kVIntakeMotor = new PrefDouble("kV Intake Roller", 0.19); //Old values from Abruticus
  }
  
  public static final class IndexerConstants {
    public static final int kIndexerMotorID = 55; //Old values from Abruticus
    public static final int kTrapMotorID = 59; //Old values from Abruticus

    public static final double kIndexerNeutralDeadband = 0.05;

    public static final PrefDouble kIndexerVelocityRPS = new PrefDouble("Indexer Velocity", 90); //Old values from Abruticus
    public static final PrefDouble kTrapVelocityRPS    = new PrefDouble("Trap Velocity",    81); //Old values from Abruticus

    public static final PrefDouble kIndexerReverseRPS = new PrefDouble("Indexer Reverse Velocity", -10); //Old values from Abruticus
    public static final double kIndexerMinVelocityRPS = -100; //Old values from Abruticus
    public static final double kIndexerMaxVelocityRPS = 100; //Old values from Abruticus

    public static final PrefDouble kIndexerVelocityIncrement = new PrefDouble("Indexer Velocity Increment", 10);

    public static final PrefDouble kPIndexerMotor = new PrefDouble("kP Indexer Pivot Motor", 0.9); //Old values from Abruticus
    public static final PrefDouble kIIndexerMotor = new PrefDouble("kI Indexer Pivot Motor", 0); //Old values from Abruticus
    public static final PrefDouble kDIndexerMotor = new PrefDouble("kD Indexer Pivot Motor", 0); //Old values from Abruticus
    public static final PrefDouble kVIndexerMotor = new PrefDouble("kV Indexer Pivot Motor", 0.12); //Old values from Abruticus

    public static final PrefDouble kPTrapMotor = new PrefDouble("kP Indexer Trap Motor", 0.9); //Old values from Abruticus
    public static final PrefDouble kITrapMotor = new PrefDouble("kI Indexer Trap Motor", 0); //Old values from Abruticus
    public static final PrefDouble kDTrapMotor = new PrefDouble("kD Indexer Trap Motor", 0); //Old values from Abruticus
    public static final PrefDouble kVTrapMotor = new PrefDouble("kV Indexer Trap Motor", 0.12); //Old values from Abruticus
  }

  public static final class TrapConstants {
    public static final int kElevatorID = 99; //tune
    public static final int kElevatorAmpPosition = 200; //tune
    public static final int kElevatorTrapPosition = 100; //tune
    public static final int kElevatorDownPosition = 0; //tune
    public static final PrefDouble kElevatorDeadband = new PrefDouble("Elevator Deadband", 0.0); // make this a real value as well
    public static final int kTrampSpeed = 5; //tune
  }

  public static final class BeamBreakSensorConstants {
    // TODO: update with real values
    public static final int kIntakeSensorId = 8;
    public static final int kShooterSensorId = 0;
    public static final int kTrampSensorId = 0;
  }

  public static class ClimbConstants {
    public static final int kLeftClimbMotorID = 54;
    public static final int kRightClimbMotorID = 57;

    public static final PrefDouble kClimbMinPosition = new PrefDouble("Climb Min Position", 0);
    public static final PrefDouble kClimbMaxPosition = new PrefDouble("Climb Max Position", 0.27);
    public static final PrefDouble kClimbPositionOffset = new PrefDouble("Climb Position Offset", 0.0);
    

    public static final PrefDouble kClimbkP = new PrefDouble("Climb Motor kP", 1);
    public static final PrefDouble kClimbkI = new PrefDouble("Climb Motor kI", 0);
    public static final PrefDouble kClimbkD = new PrefDouble("Climb Motor kD", 0.0);
    public static final PrefDouble kClimbPIDErrorTolerance = new PrefDouble("Climb Motor PID Error Tolerance", 0.01);
    public static final PrefDouble kClimbPIDErrorDerivativeTolerance = new PrefDouble("Climb Motor PID Error Derivative Tolerance", 0.01);
    public enum ClimbPostions {
      NEUTRAL,
      TOP,
      BOTTOM
    }
  }

  public static class BeambreakConstants {
    public enum BeambreakIds {
      SENSORA(0),
      SENSORB(0);

      public int id;
      BeambreakIds(int _id){
        id=_id;
      }
    }
  }

  public static class LEDConstants {
    public static final int CANdleID = 0; // TODO change later
    public static final int CANdleLength = 8;

    public static class Colors {
      public static final Color BLACK = new Color(0.0, 0.0, 0.0); // shows up as nothing
      public static final Color WHITE = new Color(1.0,1.0,1.0); 
      public static final Color RED = new Color(1.0, 0.0, 0.0); 
      public static final Color GREEN = new Color(0.0, 1.0, 0.0); 
      public static final Color BLUE = new Color(0.0, 0.0, 1.0); 
      public static final Color NERDHERD_BLUE = new Color(0.132, 0.415, 1.0); // #071635 as base, brightened fully
    }
    
    public enum LEDStrips {
      ALL(0, CANdleLength),
      CANDLE(0,8),
      ;

      public int index, count;
      LEDStrips(int _index, int _count) {
        this.index = _index;
        this.count = _count;
      }
    }
  }

}
