package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.util.NerdyMath;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeRoller extends SubsystemBase implements Reportable {

    private final TalonFX intakeMotor;
    private final TalonFXConfigurator IntakeConfigurator;
    private final VelocityVoltage VelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VoltageOut VoltageRequest = new VoltageOut(0, true, false, false, false);

    private final NeutralOut brakeRequest = new NeutralOut();
    
    private boolean enabled = true;
    public boolean velocityControl = true;

    public IntakeRoller() {
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, "rio");
        intakeMotor.setInverted(false); //check later
        IntakeConfigurator = intakeMotor.getConfigurator();

        CommandScheduler.getInstance().registerSubsystem(this);

        configureMotor();
        configurePID();
    }

    public void configureMotor() {
        TalonFXConfiguration IntakeConfigs = new TalonFXConfiguration();
        IntakeConfigurator.refresh(IntakeConfigs);

        IntakeConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IntakeConfigs.Feedback.RotorToSensorRatio = 1;
        IntakeConfigs.Feedback.SensorToMechanismRatio = 1;

        IntakeConfigs.Voltage.PeakForwardVoltage = 11.5;
        IntakeConfigs.Voltage.PeakReverseVoltage = -11.5;

        IntakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IntakeConfigs.MotorOutput.DutyCycleNeutralDeadband = IntakeConstants.kIntakeNeutralDeadband;
        IntakeConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        IntakeConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
        IntakeConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
        IntakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
        IntakeConfigs.CurrentLimits.StatorCurrentLimit = 50;
        IntakeConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        IntakeConfigs.Audio.AllowMusicDurDisable = true;

        StatusCode Result = IntakeConfigurator.apply(IntakeConfigs);
        if (!Result.isOK()) {
            DriverStation.reportError("Could not apply intake configs, error code: "+ Result.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        IntakeConstants.kIntakeVelocity.loadPreferences();

        TalonFXConfiguration IntakeMotorConfigs = new TalonFXConfiguration();
        
        intakeMotor.getConfigurator().refresh(IntakeMotorConfigs);
        IntakeConstants.kPIntakeMotor.loadPreferences();
        IntakeConstants.kIIntakeMotor.loadPreferences();
        IntakeConstants.kDIntakeMotor.loadPreferences();
        IntakeConstants.kVIntakeMotor.loadPreferences();
        IntakeMotorConfigs.Slot0.kP = 0.425; //IntakeConstants.kPIntakeMotor.get();
        IntakeMotorConfigs.Slot0.kI = 0; //IntakeConstants.kIIntakeMotor.get();
        IntakeMotorConfigs.Slot0.kD = 0; //IntakeConstants.kDIntakeMotor.get();
        IntakeMotorConfigs.Slot0.kV = 0.1; //IntakeConstants.kVIntakeMotor.get();

        StatusCode Result = intakeMotor.getConfigurator().apply(IntakeMotorConfigs);

        if (!Result.isOK()){
            DriverStation.reportError("Could not apply intake configs, error code:"+ Result.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            intakeMotor.setControl(brakeRequest);
            return;
        }

        if (Math.abs(VelocityRequest.Velocity) < 0.5) {
            VelocityRequest.Velocity = 0;
            intakeMotor.setControl(brakeRequest);
            return;
        }

        if (velocityControl) {
            intakeMotor.setControl(VelocityRequest);
            return;
        }
        
        VoltageRequest.Output = VelocityRequest.Velocity * 12 / 100;
        intakeMotor.setControl(VoltageRequest);
    }

    //****************************** VELOCITY METHODS ******************************//

    public void stop() {
        this.enabled = false;
        VelocityRequest.Velocity = 0;
        intakeMotor.setControl(brakeRequest);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void setVelocity(double velocity) {
        VelocityRequest.Velocity = velocity;
    }

    /**
     * Increment the intake's velocity.
     * @param increment
     */
    public void incrementVelocity(double increment) {
        double newVelocity = VelocityRequest.Velocity + increment;
        if ((increment > 0 && newVelocity < IntakeConstants.kIntakeMaxVelocity) ||
            (increment < 0 && newVelocity > IntakeConstants.kIntakeMinVelocity)
            ) {
            VelocityRequest.Velocity = newVelocity;
        }
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

    /**
     * Ramp the velocity within a certain timeframe
     * 
     * May take longer if the command scheduler is overrunning
     * @param initialVelocity The initial velocity in RPS
     * @param finalVelocity The final velocity in RPS
     * @param rampTimeSeconds
     * @return
     */
    public Command rampVelocity(double initialVelocity, double finalVelocity, double rampTimeSeconds) {
        final double initialVel = NerdyMath.clamp(initialVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        final double finalVel = NerdyMath.clamp(finalVelocity, ShooterConstants.kShooterMinVelocityRPS, ShooterConstants.kShooterMaxVelocityRPS);
        
        // Change in Velocity / Command Scheduler Loops (assumes 20 hz)
        final double increment = (finalVel - initialVel) / (rampTimeSeconds * 20);

        // Check whether the current velocity is over/under final velocity
        BooleanSupplier rampFinished = 
            finalVel > initialVel ? 
                () -> VelocityRequest.Velocity >= finalVel : 
                () -> VelocityRequest.Velocity <= finalVel;

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

    public Command intakeCommand() {
        return setVelocityCommand(IntakeConstants.kIntakeVelocity.get());
    }

    public Command autoIntakeCommand() {
        return setVelocityCommand(IntakeConstants.kAutoIntakeVelocity.get());
    }

    public Command outtakeCommand() {
        return setVelocityCommand(IntakeConstants.kOutakeVelocity.get());
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");

        switch (priority) {
            case ALL:
            case MEDIUM:
            tab.addDouble("Intake Position", () -> intakeMotor.getPosition().getValueAsDouble());

            case MINIMAL:
            tab.addBoolean("Intake Roller Enabled", () -> enabled);
            tab.addDouble("Intake Velocity", () -> intakeMotor.getVelocity().getValueAsDouble());
            tab.addDouble("Intake Motor Supply Current", () -> intakeMotor.getSupplyCurrent().getValueAsDouble());
            tab.addDouble("Intake Motor Stator Current", () -> intakeMotor.getStatorCurrent().getValueAsDouble());
                break;
        
            default:
                break;
        }
        

    }
    
}