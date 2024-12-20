package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class Indexer extends SubsystemBase implements Reportable {
    
    private final TalonFX indexer;

    private final TalonFXConfigurator indexerConfigurator;  

    private final VelocityVoltage indexerVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private final NeutralOut brakeRequest = new NeutralOut();

    private boolean enabled = false;

    public Indexer() {
        indexer = new TalonFX(IndexerConstants.kIndexerMotorID, SuperStructureConstants.kCANivoreBusName);

        indexerConfigurator = indexer.getConfigurator();

        CommandScheduler.getInstance().registerSubsystem(this);
        
        configureMotor();
        configurePID();
    }

    //****************************** SETUP METHODS ******************************/

    public void configureMotor() {
        TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();
        indexerConfigurator.refresh(indexerConfigs);
        indexerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        indexerConfigs.Voltage.PeakForwardVoltage = 11.5;
        indexerConfigs.Voltage.PeakReverseVoltage = -11.5;
        indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfigs.MotorOutput.DutyCycleNeutralDeadband = IndexerConstants.kIndexerNeutralDeadband;
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        indexerConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        indexerConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode indexerResponse = indexerConfigurator.apply(indexerConfigs);

        if (!indexerResponse.isOK()){
            DriverStation.reportError("Could not apply indexer configs, error code:"+ indexerResponse.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        IndexerConstants.kIndexerVelocityRPS.loadPreferences();
        IndexerConstants.kIndexerReverseRPS.loadPreferences();

        TalonFXConfiguration indexerMotorConfigs = new TalonFXConfiguration();
        
        indexerConfigurator.refresh(indexerMotorConfigs);
        IndexerConstants.kPIndexerMotor.loadPreferences();
        IndexerConstants.kIIndexerMotor.loadPreferences();
        IndexerConstants.kDIndexerMotor.loadPreferences();
        IndexerConstants.kVIndexerMotor.loadPreferences();
        indexerMotorConfigs.Slot0.kP = IndexerConstants.kPIndexerMotor.get();
        indexerMotorConfigs.Slot0.kI = IndexerConstants.kIIndexerMotor.get();
        indexerMotorConfigs.Slot0.kD = IndexerConstants.kDIndexerMotor.get();
        indexerMotorConfigs.Slot0.kV = IndexerConstants.kVIndexerMotor.get();

        StatusCode indexerResponse = indexerConfigurator.apply(indexerMotorConfigs);

        if (!indexerResponse.isOK()){
            DriverStation.reportError("Could not apply indexer configs, error code:"+ indexerResponse.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (enabled) {
            indexer.setControl(indexerVelocityRequest);
        } else {
            indexer.setControl(brakeRequest);
        }
    }

    //****************************** STATE METHODS ******************************//

    public void stop() {
        this.enabled = false;
        indexerVelocityRequest.Velocity = 0;
        indexer.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

     //****************************** VELOCITY METHODS ******************************//

    public void setVelocity(double velocity) {
        indexerVelocityRequest.Velocity = 
            NerdyMath.clamp(
                velocity, 
                IndexerConstants.kIndexerMinVelocityRPS, 
                IndexerConstants.kIndexerMaxVelocityRPS);
    }

    public void incrementVelocity(double increment) {
        setVelocity(indexerVelocityRequest.Velocity + increment);
    }

    //****************************** VELOCITY COMMANDS ******************************//

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public Command incrementVelocityCommand(double increment) {
        return Commands.runOnce(() -> incrementVelocity(increment));
    }

    public Command rampVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
        final double initialVel = NerdyMath.clamp(initialVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        final double finalVel = NerdyMath.clamp(finalVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        
        // Change in Velocity / Command Scheduler Loops (assumes 20 hz)
        final double increment = (finalVel - initialVel) / (rampTimeSeconds * 20);

        // Check whether the current velocity is over/under final velocity
        BooleanSupplier rampFinished = 
            finalVel > initialVel ? 
                () -> indexerVelocityRequest.Velocity >= finalVel : 
                () -> indexerVelocityRequest.Velocity <= finalVel;

        if (initialVel == finalVel) {
            return setVelocityCommand(finalVel);
        } 

        return 
            Commands.sequence(
                setVelocityCommand(initialVel),
                Commands.deadline(
                    Commands.waitUntil(rampFinished),
                    incrementVelocityCommand(increment)
                )
            );
    }

    public Command indexCommand() {
        return setVelocityCommand(IndexerConstants.kIndexerVelocityRPS.get());
    }

    public Command reverseIndexCommand() {
        return setVelocityCommand(IndexerConstants.kIndexerReverseRPS.get());
    }

    public Command indexToShooterCommand() {
        return setVelocityCommand(IndexerConstants.kIndexerVelocityRPS.get());
    }

    public Command indexToElevatorCommand() {
        return setVelocityCommand(IndexerConstants.kIndexerReverseRPS.get());
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");

        switch  (priority) {
            case ALL:
            case MEDIUM:
            tab.addDouble("Indexer Position", () -> indexer.getPosition().getValueAsDouble());

            case MINIMAL:
            tab.addBoolean("Indexer Subsystem Enabled", () -> enabled);
            tab.addDouble("Indexer Velocity", () -> indexer.getVelocity().getValueAsDouble());
            tab.addDouble("Indexer Stator Current", () -> indexer.getStatorCurrent().getValueAsDouble());
            tab.addDouble("Indexer Supply Current", () -> indexer.getSupplyCurrent().getValueAsDouble());
                break;
            default:
                break;

        }
        
    }
}