package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    private final LinearFilter m_voltageFilter = LinearFilter.movingAverage(50); 
    private double m_smoothedVoltage = 12.0;

    private double m_rainbowHue = 0;

    public LEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.kPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void updateState(boolean isIntaking, boolean atTarget, boolean isShooting) {
        m_smoothedVoltage = m_voltageFilter.calculate(RobotController.getBatteryVoltage());

        if (m_smoothedVoltage < 11.0) {
            setPulse(Color.kRed);
        } 
        else if (isShooting) {
            setScrollingPattern(Color.kGreen, Color.kBlack, 2);
        }
        else if (atTarget) {
            setSolid(Color.kLimeGreen);
        }
        else if (isIntaking) {
            setScrollingPattern(Color.kOrange, Color.kBlack, 3);
        }
        else {
            setRainbow();
        }
    }

    // --- Animation Helpers ---

    public void setSolid(Color color) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) m_ledBuffer.setLED(i, color);
    }

    public void setRainbow() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            final int hue = (int)((m_rainbowHue + (i * 180 / m_ledBuffer.getLength())) % 180);
            m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        m_rainbowHue = (m_rainbowHue + 3) % 180;
    }

    public void setPulse(Color color) {
        double pulse = (Math.sin(Timer.getFPGATimestamp() * 5) + 1) / 2;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, (int)(color.red * 255 * pulse), (int)(color.green * 255 * pulse), (int)(color.blue * 255 * pulse));
        }
    }

    public void setScrollingPattern(Color main, Color bg, int interval) {
        int offset = (int)(Timer.getFPGATimestamp() * 10) % interval;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, (i + offset) % interval == 0 ? main : bg);
        }
    }

    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }
}