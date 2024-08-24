package frc.robot.subsystems;
//https://github.com/frc-team-3341/2024-Crescendo-FRC-3341/blob/main/2024-Competition-Code/src/main/java/frc/robot/subsystems/Climber.java

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.ClimbConstants.ClimbPostions;

public class Climb extends SubsystemBase implements Reportable {
    private final TalonFX climbMotor;
    private final TalonFXConfigurator climbConfigurator;
    private final VelocityVoltage climbMotorVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final PIDController climbPID = new PIDController(0, 0, 0);

    private final NeutralOut brakeRequest = new NeutralOut();
    
    private ClimbPostions climbPositionState = ClimbPostions.NEUTRAL;
    private boolean enabled = true;

    public Climb() {
        climbMotor = new TalonFX(Constants.ClimbConstants.kClimbMotorID, SuperStructureConstants.kCANivoreBusName);
        climbMotor.setInverted(false); // todo reversed?????
        climbConfigurator = climbMotor.getConfigurator();
        
        configureMotor();
    }

    public void configureMotor() {
        TalonFXConfiguration climbMotorConfigs = new TalonFXConfiguration();
        climbConfigurator.refresh(climbMotorConfigs);

        // todo fill in configs

        StatusCode status = climbConfigurator.apply(climbMotorConfigs);
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply climb motor configs, error code: " + status.toString(), new Error().getStackTrace());
        }
    }

    public void configurePID() {
        // load Preferences (Constants.*Constants.property.loadPreferences())
        TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

        climbMotor.getConfigurator().refresh(climbConfiguration);

        // todo load preferences/apply to climbConfiguration
        ClimbConstants.kClimbkP.loadPreferences();
        ClimbConstants.kClimbkI.loadPreferences();
        ClimbConstants.kClimbkD.loadPreferences();
        ClimbConstants.kClimbPIDErrorTolerance.loadPreferences();
        ClimbConstants.kClimbPIDErrorDerivativeTolerance.loadPreferences();
        climbPID.setPID(ClimbConstants.kClimbkP.get(), ClimbConstants.kClimbkI.get(), ClimbConstants.kClimbkD.get());
        climbPID.setTolerance(ClimbConstants.kClimbPIDErrorTolerance.get(), ClimbConstants.kClimbPIDErrorDerivativeTolerance.get());

        StatusCode status = climbMotor.getConfigurator().apply(climbConfiguration);
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply climb motor PID configs, error code: " + status.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            climbMotor.setControl(brakeRequest);
            setPositionStateNeutral();
            return;
        }

        if(Math.abs(climbMotorVelocityRequest.Velocity) < 0.5) {
            climbMotorVelocityRequest.Velocity = 0;
            climbMotor.setControl(brakeRequest);
            setPositionStateNeutral();
            return;
        }
        
        climbMotor.setControl(climbMotorVelocityRequest);

        switch (climbPositionState) {
            case TOP:
                if (climbPID.atSetpoint()) {
                    setPositionStateNeutral();
                    return;
                }
                //ClimbConstants.kClimbMaxPosition.loadPreferences();
                setClimbVelocity(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), ClimbConstants.kClimbMaxPosition.get()));
                break;
            case BOTTOM:
                if (climbPID.atSetpoint()) {
                    setPositionStateNeutral();
                    return;
                }
                //ClimbConstants.kClimbMinPosition.loadPreferences();
                setClimbVelocity(climbPID.calculate(climbMotor.getPosition().getValueAsDouble(), ClimbConstants.kClimbMinPosition.get()));
                break;
            default:
                climbPID.reset();
                climbMotorVelocityRequest.Velocity = 0;
                climbMotor.setControl(brakeRequest);
                break;
        }
    }

    // ~~~~~~ CONTROL FUNCTIONS ~~~~~~

    public void setPositionState(ClimbPostions state) {
        climbPositionState = state;
    }

    public void setPositionStateNeutral() {
        setPositionState(ClimbPostions.NEUTRAL);
    }

    public void setPositionStateTop() {
        setPositionState(ClimbPostions.TOP);
    }

    public void setPositionStateBottom() {
        setPositionState(ClimbPostions.BOTTOM);
    }

    private void setClimbVelocity(double verbosity) {
        climbMotorVelocityRequest.Velocity = verbosity; // verbosity
    }

    // ~~~~~~ CONTROL COMMANDS ~~~~~~

    public Command setPositionStateCommand(ClimbPostions state) {
        return Commands.runOnce(() -> setPositionState(state));
    }

    public Command setPositionStateNeutralCommand() {
        return Commands.runOnce(() -> setPositionStateNeutral());
    }

    public Command setPositionStateTopCommand() {
        return Commands.runOnce(() -> setPositionStateTop());
    }

    public Command setPositionStateBottomCommand() {
        return Commands.runOnce(() -> setPositionStateBottom());
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");
        tab.addBoolean("Climb Motor Enabled", () -> enabled);
    }
}