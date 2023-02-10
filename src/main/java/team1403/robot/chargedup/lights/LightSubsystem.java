package team1403.robot.chargedup.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.RobotConfig;


public class LightSubsystem extends CougarSubsystem {
  private AddressableLED m_lights;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;

    public LightSubsystem(String lights, CougarLibInjectedParameters injectedParameters) {
        super("lights", injectedParameters);
        
        m_lights = new AddressableLED(RobotConfig.Lights.portNumber);
        m_ledBuffer = new AddressableLEDBuffer(RobotConfig.Lights.ledBufferValue);
        m_lights.setLength(m_ledBuffer.getLength());
        m_lights.setData(m_ledBuffer);
        m_lights.start();
    }
    //setYellow and setPurple rely on vison.
    public void setYellow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 239, 239, 21);
         }
    }
    public void setPurple() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 124, 16, 231);
         }
    }
    //setGreen and setRed are used for the substations.
    public void setGreen() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 8, 191, 29);
         }
    }
    public void setRed() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 248, 77, 107);
         }
    }
    public void rainbow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 100, 50);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }
}
