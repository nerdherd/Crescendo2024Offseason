package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.TrapConstants;

public class Tramp extends SubsystemBase implements Reportable{
    private final TalonFX elevator;
    private final TalonFX trampShot;

    private final VelocityVoltage trampVelocityRequest = new VelocityVoltage(0);

    private final PIDController elevatorController;

    private boolean enabled = true;

    public Tramp(){
        elevator = new TalonFX(TrapConstants.kElevatorID);
        trampShot = new TalonFX(IndexerConstants.kTrapMotorID);

        elevatorController = new PIDController(0.1, 0, 0);
    }

    @Override
    public void periodic() {
        // Math.abs(elevatorVelocityRequest.Velocity) < 0.5 || 
        if (!enabled){
            setElevatorVelocity(0);
            return;
        }

        if (elevatorController.atSetpoint()) {
            setElevatorVelocity(0);
        } else {
            setElevatorVelocity(elevatorController.calculate(elevator.getPosition().getValue()));
        }

        trampShot.setControl(trampVelocityRequest);
    }

    private void setElevatorVelocity(double velocity){
        elevator.set(velocity);
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
