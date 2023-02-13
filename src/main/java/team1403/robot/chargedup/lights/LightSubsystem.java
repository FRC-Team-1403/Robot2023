package team1403.robot.chargedup.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.wpi.WpiLimitSwitch;


public class LightSubsystem extends CougarSubsystem {
  private AddressableLED m_lights;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private WpiLimitSwitch m_coolerLimitSwitch;

    public LightSubsystem(String lights, CougarLibInjectedParameters injectedParameters) {
        super("lights", injectedParameters);
        
        m_lights = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(30);
        m_lights.setLength(m_ledBuffer.getLength());
        m_lights.setData(m_ledBuffer);
        ledBufferClear();
      }

    public void ledBufferClear(){
      for (var i = 0; i < m_ledBuffer.getLength(); i++){
        m_ledBuffer.setRGB(i, 0, 0, 0);
      }
      m_lights.setData(m_ledBuffer);
      m_lights.start();
    }
    public void setGreen() {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, m_ledBuffer.getLength()-i, m_ledBuffer.getLength() + i, m_ledBuffer.getLength() * i);
      }
         
      m_lights.setData(m_ledBuffer);
      m_lights.start();
    }

  //   public void rainbow() {
  //   // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  //   //   final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
  //   //   m_ledBuffer.setHSV(i, hue, 100, 24);
  //   // }
  //   // m_rainbowFirstPixelHue += 3;
  //   // m_rainbowFirstPixelHue %= 180;
  //     for(var i = 0; i < m_ledBuffer.getLength()/2; i=i+2){
  //       m_ledBuffer.setRGB(i, 0, 255, 0);
  //     }
  //     for(var j = 1; j < m_ledBuffer.getLength()/2; j=j+2){
  //       m_ledBuffer.setRGB(j+m_ledBuffer.getLength()/2, 255, 255, 0);
  //     }
  //   m_lights.setData(m_ledBuffer);
  //   m_lights.start();
  // }
}
