package frc.robot.subsystems;

import frc.robot.Constants.BeamBreakSensorConstants;
import frc.robot.Constants.BeamBreakSensorConstants.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BeamBreakSensor implements Reportable {
    private final int blackPort;
    private final int whitePort;
    private final DigitalInput beamBreakSensorBlack;
    private final DigitalInput beamBreakSensorWhite;

    private boolean noteDetected;
    private boolean lastBlackValue;
    private boolean lastWhiteValue;
    private boolean illegalInput = false;

    public BeamBreakSensor(Ports ports) {
        blackPort = ports.blackPort;
        whitePort = ports.whitePort;
        beamBreakSensorBlack = new DigitalInput(blackPort);
        beamBreakSensorWhite = new DigitalInput(whitePort);
    }

    public boolean noteSensed() {
        lastBlackValue = beamBreakSensorBlack.get();
        lastWhiteValue = beamBreakSensorWhite.get();
        if ((lastBlackValue && lastWhiteValue) || (!lastBlackValue && !lastWhiteValue)) {
            illegalInput = true;
            noteDetected = false;
        }

        if(!lastBlackValue && lastWhiteValue){
            noteDetected = true;
        }
        else if(lastBlackValue && !lastWhiteValue){
            noteDetected = false;
        }
        else{
            DriverStation.reportError("Fault in beam break sensor, error code: ", true);
            noteDetected = false;
        }
        return noteDetected;
    }

    public boolean noteSensedWithoutPolling() {
        return noteDetected;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addBoolean("Note Detected", this::noteSensed);
        tab.addBoolean("Beam Break Sensor Connected", () -> !illegalInput);
        tab.addBoolean("Last Black Value", () -> lastBlackValue);
        tab.addBoolean("Last White Value", () -> lastWhiteValue);
    }
    
}