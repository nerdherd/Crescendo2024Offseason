package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeambreakConstants;
import frc.robot.Constants.BeambreakConstants.BeambreakIds;

public class Beambreak extends SubsystemBase{
    private DigitalInput sensor;

    public Beambreak(BeambreakIds ids){
        sensor = new DigitalInput(ids.id);
    }

    public boolean getSensor(){
        return !sensor.get();
    }
}