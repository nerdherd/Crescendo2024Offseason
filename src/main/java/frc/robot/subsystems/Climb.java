package frc.robot.subsystems;
//https://github.com/frc-team-3341/2024-Crescendo-FRC-3341/blob/main/2024-Competition-Code/src/main/java/frc/robot/subsystems/Climber.java

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private final TalonFX leftClimbMotor;
    private final TalonFX rightClimbMotor;
    private final TalonFXConfigurator leftClimbConfigurator;
    private final TalonFXConfigurator rightClimbConfigurator;

    private final VelocityVoltage leftClimbMotorVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final VelocityVoltage rightClimbMotorVelocityRequest = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    
    private final VoltageOut leftVoltageRequest = new VoltageOut(0);
    private final VoltageOut rightVoltageRequest = new VoltageOut(0);

    private final Follower followRequest = new Follower(ClimbConstants.kLeftClimbMotorID, true);

    private final PIDController climbPID = new PIDController(0, 0, 0);

    private final NeutralOut brakeRequest = new NeutralOut();
    
    private ClimbPostions climbPositionState = ClimbPostions.NEUTRAL;

    private boolean enabled = true;
    public boolean velocityControl = true;

    public Climb() {
        leftClimbMotor = new TalonFX(Constants.ClimbConstants.kLeftClimbMotorID, SuperStructureConstants.kCANivoreBusName);
        leftClimbMotor.setInverted(false); // todo reversed?????
        leftClimbConfigurator = leftClimbMotor.getConfigurator();

        rightClimbMotor = new TalonFX(Constants.ClimbConstants.kLeftClimbMotorID, SuperStructureConstants.kCANivoreBusName);
        rightClimbConfigurator = rightClimbMotor.getConfigurator();

        rightClimbMotor.setControl(followRequest);
        
        configureMotor();
    }
    /**
     * configure motors things
     */
    public void configureMotor() {
        TalonFXConfiguration leftClimbMotorConfigs = new TalonFXConfiguration();
        leftClimbConfigurator.refresh(leftClimbMotorConfigs);

        TalonFXConfiguration rightClimbMotorConfigs = new TalonFXConfiguration();
        rightClimbConfigurator.refresh(rightClimbMotorConfigs);

        // todo fill in configs

        StatusCode status = leftClimbConfigurator.apply(leftClimbMotorConfigs);
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply left climb motor configs, error code: " + status.toString(), new Error().getStackTrace());
        }

        status = rightClimbConfigurator.apply(rightClimbMotorConfigs);
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply right climb motor configs, error code: " + status.toString(), new Error().getStackTrace());
        }
    }
    /**
     * Configure motor PIDS
     */
    public void configurePID() {
        // load Preferences (Constants.*Constants.property.loadPreferences())
        TalonFXConfiguration climbConfiguration = new TalonFXConfiguration();

        leftClimbMotor.getConfigurator().refresh(climbConfiguration);

        // todo load preferences/apply to climbConfiguration
        ClimbConstants.kClimbkP.loadPreferences();
        ClimbConstants.kClimbkI.loadPreferences();
        ClimbConstants.kClimbkD.loadPreferences();
        ClimbConstants.kClimbPIDErrorTolerance.loadPreferences();
        ClimbConstants.kClimbPIDErrorDerivativeTolerance.loadPreferences();
        climbPID.setPID(ClimbConstants.kClimbkP.get(), ClimbConstants.kClimbkI.get(), ClimbConstants.kClimbkD.get());
        climbPID.setTolerance(ClimbConstants.kClimbPIDErrorTolerance.get(), ClimbConstants.kClimbPIDErrorDerivativeTolerance.get());

        StatusCode status = leftClimbMotor.getConfigurator().apply(climbConfiguration);
        if (!status.isOK()) {
            DriverStation.reportError("Could not apply climb motor PID configs, error code: " + status.toString(), new Error().getStackTrace());
        }
    }

    @Override
    public void periodic() {
        if (!enabled) {
            leftClimbMotor.setControl(brakeRequest);
            setPositionStateNeutral();
            return;
        }

        if(Math.abs(leftClimbMotorVelocityRequest.Velocity) < 0.5) {
            leftClimbMotorVelocityRequest.Velocity = 0;
            leftClimbMotor.setControl(brakeRequest);
            setPositionStateNeutral();
            return;
        }
        
        leftClimbMotor.setControl(leftClimbMotorVelocityRequest);
        // STATE MACHINE KYLE HUANG FOREVER
        switch (climbPositionState) {
            case TOP:
                if (climbPID.atSetpoint()) {
                    setPositionStateNeutral();
                    return;
                }
                //ClimbConstants.kClimbMaxPosition.loadPreferences();
                setClimbVelocity(climbPID.calculate(leftClimbMotor.getPosition().getValueAsDouble(), ClimbConstants.kClimbMaxPosition.get()));
                break;
            case BOTTOM:
                if (climbPID.atSetpoint()) {
                    setPositionStateNeutral();
                    return;
                }
                //ClimbConstants.kClimbMinPosition.loadPreferences();
                setClimbVelocity(climbPID.calculate(leftClimbMotor.getPosition().getValueAsDouble(), ClimbConstants.kClimbMinPosition.get()));
                break;
            default:
                climbPID.reset();
                leftClimbMotorVelocityRequest.Velocity = 0;
                leftClimbMotor.setControl(brakeRequest);
                break;
        }
    }

    // ~~~~~~ CONTROL FUNCTIONS ~~~~~~
    /**
     * sets climb position state to state
     * @param state new state
     */
    public void setPositionState(ClimbPostions state) {
        climbPositionState = state;
    }
    /**
     * sets current state to neutral
     */
    public void setPositionStateNeutral() {
        setPositionState(ClimbPostions.NEUTRAL);
    }
    /**
     * sets current state to top
     */
    public void setPositionStateTop() {
        setPositionState(ClimbPostions.TOP);
    }
    /**
     * sets current state to bottom
     */
    public void setPositionStateBottom() {
        setPositionState(ClimbPostions.BOTTOM);
    }
    /**
     * sets climb velocity to verbosity (velocity)
     * ONLY USED INTERNALLY
     * @param verbosity velocity
     */
    private void setClimbVelocity(double verbosity) {
        leftClimbMotorVelocityRequest.Velocity = verbosity; // verbosity
    }

    // ~~~~~~ CONTROL COMMANDS ~~~~~~
    /**
     * sets current state to new state command
     * @param state new state
     * @return command
     */
    public Command setPositionStateCommand(ClimbPostions state) {
        return Commands.runOnce(() -> setPositionState(state));
    }
    /**
     * sets current state to neutral command
     * @return command
     */
    public Command setPositionStateNeutralCommand() {
        return Commands.runOnce(() -> setPositionStateNeutral());
    }
    /**
     * sets current state to top command
     * @return command
     */
    public Command setPositionStateTopCommand() {
        return Commands.runOnce(() -> setPositionStateTop());
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }
    /**
     * sets current state to bottom command
     * @return command
     */
    public Command setPositionStateBottomCommand() {
        return Commands.runOnce(() -> setPositionStateBottom());
    }
    public Command setVelocityCommand(double velocity) {
        return Commands.runOnce(() -> setClimbVelocity(velocity));
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Climb");

        switch (priority) {
            case ALL:
            case MEDIUM:
            tab.addDouble("Left Climb Motor Velocity", () -> leftClimbMotor.getVelocity().getValueAsDouble());
            tab.addDouble("Right Climb Motor Velocity", () -> rightClimbMotor.getVelocity().getValueAsDouble());

            case MINIMAL:
            tab.addBoolean("Climb Motor Enabled", () -> enabled);
            tab.addDouble("Left Climb Motor Position", () -> leftClimbMotor.getPosition().getValueAsDouble());
            tab.addDouble("Right Climb Motor Position", () -> rightClimbMotor.getPosition().getValueAsDouble());
            tab.addDouble("Left Motor Stator Current", () -> leftClimbMotor.getStatorCurrent().getValueAsDouble());
            tab.addDouble("Left Motor Supply Current", () -> leftClimbMotor.getSupplyCurrent().getValueAsDouble());
            tab.addDouble("Right Motor Stator Current", () -> rightClimbMotor.getStatorCurrent().getValueAsDouble());
            tab.addDouble("Right Motor Supply Current", () -> rightClimbMotor.getSupplyCurrent().getValueAsDouble());
                break;
        
            default:
                break;
        }
        

    }

}