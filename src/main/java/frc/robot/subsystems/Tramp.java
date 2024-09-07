package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TrapConstants;
import frc.robot.util.NerdyMath;

public class Tramp extends SubsystemBase implements Reportable{
    private final TalonFX elevator;
    private final TalonFX trampShot;

    private final VelocityVoltage trampVelocityRequest = new VelocityVoltage(0);

    private final PIDController elevatorController;
    // private final PIDController rollerController;

    private boolean elevatorEnabled = true;
    private boolean rollerEnabled = true;


    public Tramp(){
        elevator = new TalonFX(TrapConstants.kElevatorID);
        trampShot = new TalonFX(IndexerConstants.kTrapMotorID);

        elevatorController = new PIDController(0.1, 0, 0);
        // rollerController = new PIDController(0.1, 0, 0);
    }

    @Override
    public void periodic() {
        // Math.abs(elevatorVelocityRequest.Velocity) < 0.5 || 
        if (elevatorEnabled){
            if (elevatorController.atSetpoint()) {
                setElevatorVelocity(0);
            } else {
                setElevatorVelocity(elevatorController.calculate(elevator.getPosition().getValue()));
            }
        } else {
            setElevatorVelocity(0);
        }
        if (!rollerEnabled){
            setRollerVelocity(0);
        }
    }

    
    
    private void setElevatorVelocity(double velocity){
        elevator.set(velocity);
    }

    private void setRollerVelocity(double verbocity){
        trampShot.set(verbocity);
    }

    // state methods
    public void stop() {
        stopElevator();
        stopRoller();
    }
    public void stopElevator() {
        this.elevatorEnabled = false;
    }
    public void stopRoller() {
        this.rollerEnabled = false;
    }
 
    public Command stopElevatorCommand() {
        return Commands.runOnce(() -> stopElevator());
    }

    public Command stopRollerCommand() {
        return Commands.runOnce(() -> stopRoller());
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

 
    public void setEnabledElevator(boolean enabled) {
        this.elevatorEnabled = enabled;
    }

    public void setRollerEnabled(boolean enabled) {
        this.rollerEnabled = enabled;
    }

    public void setEnabled(boolean enabled) {
        this.rollerEnabled = enabled;
        this.elevatorEnabled = enabled;
    }


 
    public Command setEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabled(enabled));
    }

    public Command setElevatorEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setEnabledElevator(enabled));
    }

    public Command setRollerEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> setRollerEnabled(enabled));
    }

    public boolean hasReachedPosition(double positionDegrees) {
        return NerdyMath.inRange(
            elevator.getPosition().getValueAsDouble(),
            positionDegrees - TrapConstants.kElevatorDeadband.get(),
            positionDegrees + TrapConstants.kElevatorDeadband.get()
        ) && NerdyMath.inRange(
            elevator.getPosition().getValueAsDouble(),
            positionDegrees - TrapConstants.kElevatorDeadband.get(),
            positionDegrees + TrapConstants.kElevatorDeadband.get()
        );
    }

    /**
     * Sets the elevator to move to amp position
     */
    public void elevatorAmp(){
        elevatorController.setSetpoint(TrapConstants.kElevatorAmpPosition);
    }

    /**
     * Sets elevator to move to trap position
     */
    public void elevatorTrap(){
        elevatorController.setSetpoint(TrapConstants.kElevatorTrapPosition);
    }

    /**
     * Sets elevator to move to down position
     */
    public void elevatorDown(){
        elevatorController.setSetpoint(TrapConstants.kElevatorDownPosition);
    }

    /**
     * Stops the tramp launcher
     */
    public void trampStop(){
        trampVelocityRequest.Velocity = 0;
    }

    /**
     * Shoots the tramp launcher
     */
    public void trampShoot(){
        trampVelocityRequest.Velocity = TrapConstants.kTrampSpeed;
    }

    // elevator amp command
    public Command setElevatorAmpCommand() {
        return Commands.runOnce(() -> elevatorAmp());
    }

    // elevator elevator Trap
    public Command setElevatorTrapCommand() {
        return Commands.runOnce(() -> elevatorTrap());
    }

    // elevator down command
    public Command setElevatorDownCommand() {
        return Commands.runOnce(() -> elevatorDown());
    }

    // elevator tramp stop
    public Command setEtrampStopCommand() {
        return Commands.runOnce(() -> trampStop());
    }

    // elevator tramp Shoot
    public Command settrampShootCommand() {
        return Commands.runOnce(() -> trampShoot());
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportToSmartDashboard'");
    }
    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initShuffleboard'");
    }
}
