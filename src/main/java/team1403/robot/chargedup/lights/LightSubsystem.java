package team1403.robot.chargedup.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;

/**
 * The class creating the LED subsystem.
 * 
 */

public class LightSubsystem extends CougarSubsystem {
  private AddressableLED m_lights;
  private AddressableLEDBuffer m_ledBuffer;

  /**
   * Initializing the subsystem.
   */
  public LightSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("lights", injectedParameters);
        
    m_lights = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(30);
    m_lights.setLength(m_ledBuffer.getLength());
    m_lights.setData(m_ledBuffer);
    ledBufferClear();
  }

  /**
   * No light.
   * 
   */
  public void ledBufferClear() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /** 
  * Green light.
  * For shelf pick up.
  * 
  */

  public void setGreen() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 105, 0);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * 
   * Red light.
   * For floor pick up.
   * 
   */
  
  public void setRed() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 47);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * Purple light.
   * For cubes.
   * 
   */

  public void setPurple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 107, 3, 252);
    }
       
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * Yellow light.
   * For cones.
   * 
   */

  public void setYellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 230, 0);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /** 
  * Light blue light.
  * for when the arm is tucked.
  *
  */

  public void setBlue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 166, 255);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * Dark blue light.
   * when the arm is fully extended.
   * 
   */

  public void setDeepBlue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 13, 255);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * Pink light.
   * For when the arm is low.
   * 
   */

  public void setPink() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 13, 255);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }
}
