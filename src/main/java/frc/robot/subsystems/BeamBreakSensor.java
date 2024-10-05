package frc.robot.subsystems;

// import frc.robot.Constants.BeamBreakSensorConstants;
// import frc.robot.Constants.BeamBreakSensorConstants.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BeamBreakSensor implements Reportable {
    private final int port;
    private final DigitalInput beamBreakSensor;

    private boolean noteDetected;

    public BeamBreakSensor(int portIn) {
        port = portIn;
        beamBreakSensor = new DigitalInput(port);
    }

    public boolean noteSensed() {
        noteDetected = !beamBreakSensor.get();
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
    }
    
}