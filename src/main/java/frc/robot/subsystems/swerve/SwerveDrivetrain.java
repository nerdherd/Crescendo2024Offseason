package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.CANCoderConstants;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.util.NerdyMath;
import frc.robot.subsystems.Reportable;

import static frc.robot.Constants.PathPlannerConstants.kPPMaxVelocity;
import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;
import static frc.robot.Constants.SwerveDriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveDrivetrain extends SubsystemBase implements Reportable {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final Gyro gyro;
    // private final SwerveDriveOdometry odometer;
    private boolean isTest = false;
    private final SwerveDrivePoseEstimator poseEstimator;
    private DRIVE_MODE driveMode = DRIVE_MODE.FIELD_ORIENTED;

    public enum DRIVE_MODE {
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        AUTONOMOUS
    }

    /**
     * Construct a new {@link SwerveDrivetrain}
     */
    public SwerveDrivetrain(Gyro gyro) throws IllegalArgumentException {
        frontLeft = new SwerveModule(
            kFLDriveID,
            kFLTurningID,
            kFLDriveReversed,
            kFLTurningReversed,
            CANCoderConstants.kFLCANCoderID,
            CANCoderConstants.kFLCANCoderReversed);
        frontRight = new SwerveModule(
            kFRDriveID,
            kFRTurningID,
            kFRDriveReversed,
            kFRTurningReversed,
            CANCoderConstants.kFRCANCoderID,
            CANCoderConstants.kFRCANCoderReversed);
        backLeft = new SwerveModule(
            kBLDriveID,
            kBLTurningID,
            kBLDriveReversed,
            kBLTurningReversed,
            CANCoderConstants.kBLCANCoderID,
            CANCoderConstants.kBLCANCoderReversed);
        backRight = new SwerveModule(
            kBRDriveID,
            kBRTurningID,
            kBRDriveReversed,
            kBRTurningReversed,
            CANCoderConstants.kBRCANCoderID,
            CANCoderConstants.kBRCANCoderReversed);

        this.gyro = gyro;

        this.poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO: Set pose estimator weights
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); 

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry, 
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new HolonomicPathFollowerConfig(
                kPPTranslationPIDConstants, 
                kPPRotationPIDConstants, 
                kPPMaxVelocity,
                kTrackWidth,
                new ReplanningConfig()), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
    }


    /**
     * Have modules move towards states and update odometry
     */
    @Override
    public void periodic() {
        if (!isTest) {
            runModules();
        }
        
        poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    }
    
    //****************************** RESETTERS ******************************/
    /**
     * Resets the odometry to given pose 
     * @param pose  A Pose2D representing the pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometryWithAlliance(Pose2d pose){
        if (RobotContainer.IsRedSide()) {
            resetOdometry(GeometryUtil.flipFieldPose(pose));
        } else {
            resetOdometry(pose);
        }
    }

    public void zeroGyroAndPoseAngle() {
        gyro.zeroHeading();
        gyro.setOffset(0);
        Pose2d pose = getPose();
        Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), RobotContainer.IsRedSide() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), newPose);
    }

    public void resetGyroFromPoseWithAlliance(Pose2d pose) {
        if (RobotContainer.IsRedSide()) {
            double angle = GeometryUtil.flipFieldPose(pose).getRotation().getDegrees() - 180;
            angle = NerdyMath.posMod(angle, 360);
            // gyro.resetHeading(angle);
        } else {
            // gyro.resetHeading(NerdyMath.posMod(pose.getRotation().getDegrees(), 360));
        }
    }

    public void refreshModulePID() {
        frontLeft.refreshPID();
        backLeft.refreshPID();
        frontRight.refreshPID();  
        backRight.refreshPID();
    }

    /**
     * Stops all modules. See {@link SwerveModule#stop()} for more info.
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Have modules move to their desired states. See {@link SwerveModule#run()} for more info.
     */
    public void runModules() {
        frontLeft.run();
        frontRight.run();
        backLeft.run();
        backRight.run();
    }

    //****************************** GETTERS ******************************/

    public Gyro getImu() {
        return this.gyro;
    }

    /**
     * Gets a pose2d representing the position of the drivetrain
     * @return A pose2d representing the position of the drivetrain
     */
    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Get the position of each swerve module
     * @return An array of swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(), 
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    private ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, turnSpeed)
            )
        );
    }

    public void drive(double xSpeed, double ySpeed) {
        drive(xSpeed, ySpeed, 0);
    }

    public void driveFieldOriented(double xSpeed, double ySpeed, double turnSpeed) {
        setModuleStates(
            SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,gyro.getRotation2d() )
            )
        );
    }

    public void driveFieldOriented(double xSpeed, double ySpeed) {
        driveFieldOriented(xSpeed, ySpeed, 0);
    }

    public Command driveToPose(Pose2d destPoseInBlue, double maxVelocityMps, double maxAccelerationMpsSq) {
        PathConstraints pathcons = new PathConstraints(
            maxVelocityMps, maxAccelerationMpsSq, 
            Units.degreesToRadians(180), Units.degreesToRadians(360)
        );
        return Commands.either(
            AutoBuilder.pathfindToPose(GeometryUtil.flipFieldPose(destPoseInBlue), pathcons),
            AutoBuilder.pathfindToPose(destPoseInBlue, pathcons),
            RobotContainer::IsRedSide  
        );
    }
    
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(targetStates);
    }


    //****************************** SETTERS ******************************/

    /**
     * Set the drive mode (only for telemetry purposes)
     * @param driveMode
     */
    public void setDriveMode(DRIVE_MODE driveMode) {
        this.driveMode = driveMode;
    }

    public void setVelocityControl(boolean withVelocityControl) {
        frontLeft.toggleVelocityControl(withVelocityControl);
        frontRight.toggleVelocityControl(withVelocityControl);
        backLeft.toggleVelocityControl(withVelocityControl);
        backRight.toggleVelocityControl(withVelocityControl);
    }

    /**
     * Set the neutral modes of all modules.
     * <p>
     * true sets break mode, false sets coast mode
     * 
     * @param breaking  Whether or not the modules should be in break
     */
    public void setBreak(boolean breaking) {
        frontLeft.setBreak(breaking);
        frontRight.setBreak(breaking);
        backLeft.setBreak(breaking);
        backRight.setBreak(breaking);
    }

    /**
     * Sets module desired states
     * @param desiredStates desired states of the four modules (FL, FR, BL, BR)
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void towModules() {
        frontLeft.setDesiredState(towModuleStates[0], false);
        frontRight.setDesiredState(towModuleStates[1], false);
        backLeft.setDesiredState(towModuleStates[2], false);
        backRight.setDesiredState(towModuleStates[3], false);
    }

    public Command towCommand() {
        return Commands.runOnce(this::towModules, this);
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Swerve");
        }

        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addString(("Current Command"), () -> {
                    Command currCommand = this.getCurrentCommand();
                    if (currCommand == null) {
                        return "null";
                    } else {
                        return currCommand.getName();
                    }
                }
                );
                tab.add("Toggle Test", Commands.runOnce(() -> isTest = !isTest));
                tab.addBoolean("Test Mode", () -> isTest);
                // Might be negative because our swerveDriveKinematics is flipped across the Y axis
            case MEDIUM:
            case MINIMAL:
                tab.addNumber("X Position (m)", () -> poseEstimator.getEstimatedPosition().getX());
                tab.addNumber("Y Position (m)", () -> poseEstimator.getEstimatedPosition().getY());
                tab.addNumber("Odometry Angle", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
                tab.addString("Drive Mode", () -> this.driveMode.toString());
                break;
        }
    }

    public void initModuleShuffleboard(LOG_LEVEL level) {
        frontRight.initShuffleboard(level);
        frontLeft.initShuffleboard(level);
        backLeft.initShuffleboard(level);
        backRight.initShuffleboard(level);
    }

    /**
     * Report values to smartdashboard.
     */
     public void reportToSmartDashboard(LOG_LEVEL level) {
    //     switch (level) {
    //         case OFF:
    //             break;
    //         case ALL:
    //         case MEDIUM:
    //             SmartDashboard.putData("Zero Modules", Commands.runOnce(this::zeroModules));
    //         case MINIMAL:
    //             SmartDashboard.putNumber("Odometer X Meters", poseEstimator.getEstimatedPosition().getX());
    //             SmartDashboard.putNumber("Odometer Y Meters", poseEstimator.getEstimatedPosition().getY());
    //             SmartDashboard.putString("Drive Mode", this.driveMode.toString());
    //             break;
    //     }
     }

    // public void reportModulesToSmartDashboard(LOG_LEVEL level) {
    //     frontRight.reportToSmartDashboard(level);
    //     frontLeft.reportToSmartDashboard(level);
    //     backLeft.reportToSmartDashboard(level);
    //     backRight.reportToSmartDashboard(level);
    // }
}