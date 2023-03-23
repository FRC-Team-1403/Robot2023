package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.StateManager.LED;

/**
 * The class creating the LED subsystem.
 * 
 */
public class LightSubsystem extends CougarSubsystem {
  private AddressableLED m_lightsLeft;
  private AddressableLED m_lightsRight;
  private AddressableLEDBuffer m_ledBufferLeft;
  private AddressableLEDBuffer m_ledBufferRight;
  private double m_rainbowFirstPixelHue;

  private LED previousLED;

  /**
   * Initializing the subsystem.
   */
  public LightSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("lights", injectedParameters);
    m_lightsLeft = new AddressableLED(0);
    m_lightsRight = new AddressableLED(2);
    m_ledBufferLeft = new AddressableLEDBuffer(60);
    m_ledBufferRight = new AddressableLEDBuffer(60);
    m_lightsRight.setLength(m_ledBufferRight.getLength());
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsLeft.setLength(m_ledBufferLeft.getLength());
    m_lightsLeft.setData(m_ledBufferLeft);

    previousLED = LED.NONE;
    ledBufferClear();
  }

  /**
   * No light.
   * 
   */
  private void ledBufferClear() {
    for (var i = 0; i < m_ledBufferLeft.getLength(); i++) {
      m_ledBufferLeft.setRGB(i, 0, 0, 0);
      m_ledBufferRight.setRGB(i, 0, 0, 0);
    }
    m_lightsLeft.setData(m_ledBufferLeft);
    m_lightsLeft.start();
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsRight.start();
  }

  /**
   * Purple light.
   * For cubes.
   * 
   */
  private void setPurple() {
    for (var i = 0; i < m_ledBufferLeft.getLength(); i++) {
      m_ledBufferLeft.setRGB(i, 107, 3, 252);
      m_ledBufferRight.setRGB(i, 107, 3, 252);
    }

    m_lightsLeft.setData(m_ledBufferLeft);
    m_lightsLeft.start();
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsRight.start();
  }

  /**
   * Yellow light.
   * For cones.
   * 
   */
  private void setYellow() {
    for (var i = 0; i < m_ledBufferLeft.getLength(); i++) {
      m_ledBufferLeft.setRGB(i, 255, 230, 0);
      m_ledBufferRight.setRGB(i, 255, 230, 0);
    }
    m_lightsLeft.setData(m_ledBufferLeft);
    m_lightsLeft.start();
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsRight.start();
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBufferLeft.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (int) ((m_rainbowFirstPixelHue + (i * 180 / m_ledBufferLeft.getLength())) % 180);
  
      // Set the value
      m_ledBufferLeft.setHSV(i, hue, 255, 128);
      m_ledBufferRight.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_lightsLeft.setData(m_ledBufferLeft);
    m_lightsLeft.start();
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsRight.start();
  }

  private void monty() {
    for (var i = 0; i < m_ledBufferLeft.getLength(); i = i + 2) {
      m_ledBufferLeft.setRGB(i, 255, 255, 0);
    }
    for (var i = 0; i < m_ledBufferRight.getLength(); i = i + 2) {
      m_ledBufferRight.setRGB(i, 255, 255, 0);
    }   
   m_lightsLeft.setData(m_ledBufferLeft);
    m_lightsRight.setData(m_ledBufferRight);
    m_lightsLeft.start();
    m_lightsRight.start();
    for (var i = 1; i < m_ledBufferLeft.getLength(); i = i + 2) {
      m_ledBufferLeft.setRGB(i, 0, 255, 0);
    } 
    for (var i = 1; i < m_ledBufferRight.getLength(); i = i + 2) {
      m_ledBufferRight.setRGB(i, 0, 255, 0);
    } 
  }

  private void setLEDColor(LED led) {
    switch (led) {
      case YELLOW: {
        setYellow();
        break;
      }
      case PURPLE: {
        setPurple();
        break;
      }
      case RAINBOW: {
        rainbow();
        break;
      }
      case MONTY: {
        monty();
        break;
      }
      default: {
        ledBufferClear();
        break;
      }
    }
    previousLED = led;
  }

  @Override
  public void periodic() {
    if (previousLED != StateManager.getInstance().getLEDState()) {
      setLEDColor(StateManager.getInstance().getLEDState());
    }
  }
}