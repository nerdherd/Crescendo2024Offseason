package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeambreakConstants;

public class Beambreak extends SubsystemBase{
    private DigitalInput sensorA;
    private DigitalInput sensorB;

    public Beambreak(){
        sensorA = new DigitalInput(BeambreakConstants.kSensorAId);
        sensorB = new DigitalInput(BeambreakConstants.kSensorBId);
    }

    public boolean getSensorA(){
        return !sensorA.get();
    }

    public boolean getSensorB(){
        return !sensorB.get();
    }
}