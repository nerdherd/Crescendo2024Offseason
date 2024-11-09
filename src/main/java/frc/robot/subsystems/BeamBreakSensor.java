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