package frc.robot.subsystems;
 
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.ShooterConstants;

/**
 * The single motor drive for the shooter's pivot
 */
public class ShooterPivot extends SubsystemBase implements Reportable {
    private final TalonFX pivot;
    private final TalonFXConfigurator pivotConfigurator;
    private final DutyCycleEncoder throughBore;
    private Pigeon2 pigeon;
 
    // Whether the pivot is running
    private boolean enabled = true;
 
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(ShooterConstants.kFullStowPosition.get() / 360, true, 0, 0, false, false, false);
    private final NeutralOut brakeRequest = new NeutralOut(); 
    public ShooterPivot() {
        pivot = new TalonFX(ShooterConstants.kLeftPivotMotorID, SuperStructureConstants.kCANivoreBusName);
        throughBore = new DutyCycleEncoder(ShooterConstants.kThroughBorePort);
        pivotConfigurator = pivot.getConfigurator();
 
        CommandScheduler.getInstance().registerSubsystem(this);
 
        configureMotor();
        configurePID();
        pigeon = new Pigeon2(ShooterConstants.kShooterPigeonID);
    }
   
    //****************************** SETUP METHODS ******************************/
    /**
     * Called once on construction for motor config initialization
     */
    public void configureMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfigs);
        pivotConfigs.Feedback.FeedbackRemoteSensorID = ShooterConstants.kShooterPigeonID;
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemotePigeon2_Roll;
        pivotConfigs.Feedback.RotorToSensorRatio = -ShooterConstants.kPivotGearRatio / 360;
        pivotConfigs.Feedback.SensorToMechanismRatio = -1;
        pivotConfigs.Feedback.FeedbackRemoteSensorID = ShooterConstants.kShooterPigeonID;
        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemotePigeon2_Roll;
        pivotConfigs.Feedback.RotorToSensorRatio = -ShooterConstants.kPivotGearRatio / 360;
        pivotConfigs.Feedback.SensorToMechanismRatio = -1;
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       
        pivotConfigs.Voltage.PeakForwardVoltage = 11.5;
        pivotConfigs.Voltage.PeakReverseVoltage = -11.5;
       
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ShooterConstants.kShooterNeutralDeadband;
 
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        pivotConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = 100;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigs.Audio.AllowMusicDurDisable = true;
 
        StatusCode statusPivot = pivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }
    }

    /**
     * Called once on construction for motor PID config initialization
     */
    public void configurePID() {
        ShooterConstants.kSpeakerPosition.loadPreferences();
        ShooterConstants.kSpeakerPosition2.loadPreferences();
        ShooterConstants.kAmpPosition.loadPreferences();
        ShooterConstants.kHandoffPosition.loadPreferences();
        ShooterConstants.kNeutralPosition.loadPreferences();
        ShooterConstants.kPrepClimbPosition.loadPreferences();
 
 
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfigs);
        ShooterConstants.kPPivotMotor.loadPreferences();
        ShooterConstants.kIPivotMotor.loadPreferences();
        ShooterConstants.kDPivotMotor.loadPreferences();
        ShooterConstants.kVPivotMotor.loadPreferences();
        ShooterConstants.kSPivotMotor.loadPreferences();
        ShooterConstants.kAPivotMotor.loadPreferences();
        ShooterConstants.kGPivotMotor.loadPreferences();
        ShooterConstants.kCruiseVelocity.loadPreferences();
        ShooterConstants.kCruiseAcceleration.loadPreferences();
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.Slot0.kP = ShooterConstants.kPPivotMotor.get();
        pivotConfigs.Slot0.kI = ShooterConstants.kIPivotMotor.get();
        pivotConfigs.Slot0.kD = ShooterConstants.kDPivotMotor.get();
        pivotConfigs.Slot0.kV = ShooterConstants.kVPivotMotor.get();
        pivotConfigs.Slot0.kS = ShooterConstants.kSPivotMotor.get();
        pivotConfigs.Slot0.kA = ShooterConstants.kAPivotMotor.get();
        pivotConfigs.Slot0.kG = ShooterConstants.kGPivotMotor.get();
        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration   = ShooterConstants.kCruiseAcceleration.get();
 
        StatusCode statusPivot = pivotConfigurator.apply(pivotConfigs);
        if (!statusPivot.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusPivot.toString(), new Error().getStackTrace());
        }
    }
 
    /**
     * Resets encoder and sets pivot position based on constant offset
     */
    public void syncEncoder() {
        // Save a consistent position offset
        ShooterConstants.kPivotOffset.loadPreferences();
        // Throughbore uses revolutions, so divide by 360
        throughBore.setPositionOffset(ShooterConstants.kPivotOffset.get() / 360);
 
        // TalonFX uses revolutions
        double position = getAbsolutePositionRev();
        position = mapRev(position);
 
        pivot.setPosition(position);
    }
 
    /**
     * Syncs throughBore encoder with constant pivot offset
     */
    public void syncAbsoluteEncoderToPigeon() {
        throughBore.reset();
        throughBore.setPositionOffset(throughBore.getPositionOffset() + getPositionRev());
 
        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset() * 360);
    }
 
    /**
     * Zero the through bore encoder and update the internal encoder
     * ONLY RUN FOR DEBUGGING
     */
    public void zeroAbsoluteEncoder() {
        throughBore.reset();
        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset() * 360);
 
        // Save new offset to Preferences
        ShooterConstants.kPivotOffset.uploadPreferences();
        syncEncoder();
    }
 
    /**
     * Zeroes encoder assuming currently at full stow
     */
    public void zeroAbsoluteEncoderFullStow() {
        throughBore.reset();
        ShooterConstants.kFullStowPosition.loadPreferences();
 
        // The value is saved as degrees, but the throughbore reads it as revs
        throughBore.setPositionOffset((throughBore.getPositionOffset() + (ShooterConstants.kFullStowPosition.get() / 360)) % 1);
 
        ShooterConstants.kPivotOffset.set(throughBore.getPositionOffset() * 360);
        ShooterConstants.kPivotOffset.uploadPreferences();
 
        syncEncoder();
    }
   
    /**
     * Called every 20 ms, checks for being enabled or disabled
     */
    @Override
    public void periodic() {
        if (ShooterConstants.fullDisableShooter.get()) {
            pivot.setControl(brakeRequest);
            enabled = false;
            return;
        }
 
        if (enabled) {
            pivot.setControl(motionMagicRequest);
        } else {
            pivot.setControl(brakeRequest);
        }
    }
 
    /**
     * Maps the value to the range (-0.25, 0.75]
     * @param rev Revolutions (any range, negative or positive)
     */
    public double mapRev(double rev) {
        rev = rev - Math.floor(rev);
        if (rev > 0.75) rev -= 1.0;
        return rev;
    }
 
    /**
     * Maps the value to the range (-180, 180]
     * @param deg Degrees (any range, negative or positive)
     */
    public double mapDegrees(double deg) {
        deg = deg - (Math.floor(deg / 360.0) * 360.0);
        if (deg > 180) deg -= 360;
        return deg;
    }
 
    /**
     * Sets and updates break mode
     * @param breaking to break or not to break
     */
    public void setBreakMode(boolean breaking) {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
        pivotConfigurator.refresh(pivotConfigs);
        pivotConfigs.MotorOutput.NeutralMode = (breaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        pivotConfigurator.apply(pivotConfigs);
    }
 
    //****************************** STATE METHODS ******************************/
 
    /**
     * @return target position according too motionMagic
     */
    public double getTargetPositionRev() {
        return motionMagicRequest.Position;
    }
 
    /**
     * @return getTargetPositionRev in degrees
     */
    public double getTargetPositionDegrees() {
        return getTargetPositionRev() * 360;
    }
 
    /**
     * @return current position according to motor encoders
     */
    public double getPositionRev() {
        return pivot.getPosition().getValueAsDouble();
    }
 
    /**
     * @return getPositionRev in degrees
     */
    public double getPositionDegrees() {
        return getPositionRev() * 360;
    }
 
    /**
     * @return absolute position, offset by a constant (accounting for inversion)
     */
    public double getAbsolutePositionRev() {
        if (ShooterConstants.kPivotAbsoluteEncoderInverted) {
            return mapRev(throughBore.getPositionOffset() - throughBore.getAbsolutePosition());
        }
        return mapRev(throughBore.getAbsolutePosition() - throughBore.getPositionOffset());
    }
 
    /**
     * @return getAbsolutePositionRev in degrees
     */
    public double getAbsolutePositionDegrees() {
        return getAbsolutePositionRev() * 360;
    }
 
    /** 
     * Checks whether the pivot is within the deadband for a position
     * @param positionDegrees position to check for reachment
     */
    public boolean hasReachedPosition(double positionDegrees) {
        return NerdyMath.inRange(
            getPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadband.get(),
            positionDegrees + ShooterConstants.kPivotDeadband.get()
        ) && NerdyMath.inRange(
            getTargetPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadband.get(),
            positionDegrees + ShooterConstants.kPivotDeadband.get()
        );
    }
    
    /**
     * Same as hasReachedPosition, but with kPivotDeadbandClose from ShooterConstants
     * @param positionDegrees
     * @return
     */
    public boolean hasReachedPositionAccurate(double positionDegrees) {
        return NerdyMath.inRange(
            getPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadbandClose.get(),
            positionDegrees + ShooterConstants.kPivotDeadbandClose.get()
        ) && NerdyMath.inRange(
            getTargetPositionDegrees(),
            positionDegrees - ShooterConstants.kPivotDeadbandClose.get(),
            positionDegrees + ShooterConstants.kPivotDeadbandClose.get()
        );
    }
 
    /** 
     * Check if the shooter is in a safe position for the intake to move
     */
    public boolean hasReachedNeutral() {
        return hasReachedPosition(ShooterConstants.kNeutralPosition.get());
    }
    
    /** 
     * Checks if the pivot is within deadband of the target pos
     */
    public boolean atTargetPosition() {
        return hasReachedPosition(getTargetPositionDegrees());
    }
    
    
    /**
     * Checks if the pivot is within deadband of the target pos
     */
    public boolean atTargetPositionAccurate() {
        return hasReachedPositionAccurate(getTargetPositionDegrees());
    }
    /**
     * stop and disable
     */
    public void stop() {
        motionMagicRequest.Position = getPositionRev();
        enabled = false;
        pivot.setControl(brakeRequest);
    }
    /**
     * @return stop function as a command
     */
    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }
    /**
     * set enabled 
     * @param enabled On or off
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
    /**
     * @param enabled on or off
     * @return return enabled function as command
     */
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
   
    //****************************** POSITION METHODS ******************************//
 
    public void setPosition(double positionDegrees) {
        double newPos = NerdyMath.clamp(
                mapDegrees(positionDegrees),
                ShooterConstants.kPivotMinPos,
                ShooterConstants.kPivotMaxPos) / 360;
        motionMagicRequest.Position = newPos;
        SmartDashboard.putNumber("Shooter New Position", newPos);
        SmartDashboard.putNumber("Shooter New Position", newPos * 360);
    }
 
    public Command setPositionCommand(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }
 
    public void incrementPosition(double incrementDegrees) {
        SmartDashboard.putNumber("increment", incrementDegrees);
        if (Math.abs(incrementDegrees) <= 0.001) {
            return;
        }
        setPosition(getTargetPositionDegrees() + incrementDegrees);  
    }
 
    public Command incrementPositionCommand(double increment) {
        return Commands.runOnce(() -> incrementPosition(increment));
    }
 
    //****************************** POSITION COMMANDS *****************************//
 
    public Command moveToNeutral() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kNeutralPosition.get()));
    }
 
    public Command moveToAmp() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kAmpPosition.get()));
    }
 
    public Command moveToSpeaker() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPosition.get()));
    }
 
    public Command moveToSpeakerAuto() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPositionAuto.get()));
    }
 
    public Command moveToSpeakerAutoStart() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPositionAutoStart.get()));
    }
    public Command moveToSpeakerAutoStart2() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPositionAutoStart2.get()));
    }
 
    public Command moveToSpeakerFar() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPosition2.get()));
    }
    public Command moveToSpeakerFar2() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kSpeakerPosition3.get()));
    }
 
    public Command moveToHandoff() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kHandoffPosition.get()));
    }
 
    public Command moveToAutoHandoff() {
        return Commands.runOnce(() -> setPosition(ShooterConstants.kHandoffPosition2.get()));
    }
 
    //****************************** LOGGING METHODS ******************************//
 
    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}
 
    @Override
    public void initShuffleboard(LOG_LEVEL level) {
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Shooter");
        }
 
        switch (level) {
            case ALL:
                tab.addString("Current Command", () -> this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
            case MEDIUM:
            case MINIMAL:
                tab.addDouble("Shooter Desired Position", this::getTargetPositionDegrees);
                tab.addDouble("Pivot Position", this::getPositionDegrees);
                tab.addDouble("Pivot Velocity (DPS)", () -> pivot.getVelocity().getValueAsDouble() * 360);
                tab.addDouble("Pivot Applied Voltage", () -> pivot.getMotorVoltage().getValueAsDouble());
                tab.addDouble("Pivot Stator Current", () -> pivot.getStatorCurrent().getValueAsDouble());
                tab.addDouble("Pivot Supply Current", () -> pivot.getSupplyCurrent().getValueAsDouble());
 
                tab.add("Zero Absolute Encoder", Commands.runOnce(this::zeroAbsoluteEncoder));
                tab.add("Full Stow Absolute Encoder", Commands.runOnce(this::zeroAbsoluteEncoderFullStow));
                tab.add("Sync Encoder", Commands.runOnce(this::syncEncoder));
 
                tab.addDouble("Absolute Encoder Position", this::getAbsolutePositionDegrees);
                tab.addBoolean("Shooter Enabled", () -> enabled);
                tab.addDouble("Pigeon Yaw", () -> pigeon.getYaw().getValueAsDouble());
                tab.addDouble("Pigeon Pitch",() ->  pigeon.getPitch().getValueAsDouble());
                tab.addDouble("Pigeon Roll", () -> pigeon.getRoll().getValueAsDouble());
                break;
            default:
                break;
        }
    }
 
}  
 
