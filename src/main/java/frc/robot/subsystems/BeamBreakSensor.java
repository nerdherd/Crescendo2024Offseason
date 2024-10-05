package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BeamBreakSensor implements Reportable {
    private final int port;
    private final DigitalInput beamBreakSensor;


    public BeamBreakSensor(int portIn) {
        port = portIn;
        beamBreakSensor = new DigitalInput(port);
    }

    // Commented out because this code doesn't do anything usefull and is
    // inneficient

    // noteSensed is constatly called by the shuffleboard, so noteDetected
    // is constantly updated, making noteSensedWithoutPolling useless

    // That means there isn't any use to actually store if a note is 
    // detected. We should delete these comments and codesoon, I've 
    // left them in just in case I am wrong.

    // private boolean noteDetected;

    // public boolean noteSensed() {
    //     noteDetected = !beamBreakSensor.get();
    //     return noteDetected;
    // }

    // public boolean noteSensedWithoutPolling() {
    //     return noteDetected;
    // }

    public boolean noteSensed(){
        return !beamBreakSensor.get();
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {}

    @Override
    public void initShuffleboard(LOG_LEVEL priority) {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.addBoolean("Note Detected (port "+port+")", this::noteSensed);
    }
    
}