package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStrip;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(Constants.LEDConstants.CANdleID, "rio");
    private Color[] stripColors = new Color[LEDConstants.CANdleLength];

    private State state = State.DISABLED;
    public enum State {
        DISABLED,
        TELEOP,
        AUTO,
        DISCONNECTED,
        HAS_NOTE, // Note is in indexer
        SHOOTING
    }

    public LED() {
        candle.clearAnimation(0);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100); 
        
        for (int i = 0; i < stripColors.length; i++) stripColors[i] = new Color(0, 0, 0);
    }

    private void setLED(int r, int g, int b, int index) {
        candle.setLEDs(r, g, b, 0, index, 0);
    }
    private void setLED(Color color, int index) {
        candle.setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), 0, index, 0);
    }

    private void setLEDs(int r, int g, int b, int index, int count) {
        candle.setLEDs(r, g, b, 0, index, count);
    }
    private void setLEDs(int r, int g, int b, LEDStrip section) {
        setLEDs(r, g, b, section.index, section.count);
    }
    private void setLEDs(Color color, int index, int count) {
        candle.setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), 0, index, count);
    }
    private void setLEDs(Color color, LEDStrip section) {
        setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), section.index, section.count);
    }
    
    private void setLEDs(int r, int g, int b) {
        setLEDs(r,g,b);
    }
    private void setLEDs(Color color) {
        setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
    }

    public void setState(State _state){
        state = _state;
    }
    public Command setStateCommand(State _state){
        return Commands.runOnce(()->setState(_state));
    }
    public Command setStateDisabledCommand(){
        return Commands.runOnce(()->setState(State.DISABLED));
    }
    public Command setStateTeleopCommand(){
        return Commands.runOnce(()->setState(State.TELEOP));
    }
    public Command setStateAutoCommand(){
        return Commands.runOnce(()->setState(State.AUTO));
    }
    public Command setStateDisconnectedCommand(){
        return Commands.runOnce(()->setState(State.DISCONNECTED));
    }
    public Command setStateHasNoteCommand(){
        return Commands.runOnce(()->setState(State.HAS_NOTE));
    }
    public Command setStateShooterReadyCommand(){
        return Commands.runOnce(()->setState(State.SHOOTING));
    }
    
    /**
     * sets section to pattern
     * @param colors pattern to set
     * @param index index of section
     * @param count count of section
     */
    private void setStrip(Color[] colors, int index, int count) {
        for (int i = 0; i < count; i++) {
            stripColors[index + i] = colors[i % colors.length];
        }
    }
    /**
     * setStrip based on section
     * @param colors
     * @param section
     */
    private void setStrip(Color[] colors, LEDStrip section){
        setStrip(colors, section.index, section.count);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return candle.getBusVoltage(); }
    public double get5V() { return candle.get5VRailVoltage(); }
    public double getCurrent() { return candle.getCurrent(); }
    public double getTemperature() { return candle.getTemperature(); }
    public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }
    
    /**
     * applies strip colors
     */
    private void updateCANdle() {
        for (int i = 0; i < stripColors.length; i++) {
            setLED(stripColors[i], i);
        }
    }

    private int delay = 0;
    // Runs 5 times a second
    @Override
    public void periodic() {
        delay++;
        if (delay >= 10) {
            delay = 0;

            // TODO draws go here

            updateCANdle();
        }
    }
}