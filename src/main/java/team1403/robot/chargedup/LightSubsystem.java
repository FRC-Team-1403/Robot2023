package team1403.robot.chargedup;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.StateManager.LED;

/**
 * The class creating the LED subsystem.
 * 
 */

public class LightSubsystem extends CougarSubsystem {
  private AddressableLED m_lights;
  private AddressableLEDBuffer m_ledBuffer;
  private double m_rainbowFirstPixelHue;

  /**
   * Initializing the subsystem.
   */
  public LightSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("lights", injectedParameters);
        
    m_lights = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_lights.setLength(m_ledBuffer.getLength());
    m_lights.setData(m_ledBuffer);
    ledBufferClear();

    
  }

  /**
   * No light.
   * 
   */
  private void ledBufferClear() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  /**
   * Purple light.
   * For cubes.
   * 
   */

  private void setPurple() {
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

  private void setYellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 230, 0);
    }
         
    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }



  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (int) ((m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180);
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_lights.setData(m_ledBuffer);
    m_lights.start();
  }

  private void monty() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      for (var j = 0; j < 3; j++) {
      final int hue[] = {0,0};
      m_ledBuffer.setHSV(i, hue[j], 255, 128);
      }
    }
    m_lights.setData(m_ledBuffer);
    m_lights.start();

    //right yellow left not
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
        case NONE: {
            ledBufferClear();
            break;
        }
    }
  }

  @Override
  public void periodic() {
    setLEDColor(StateManager.getInstance().getLEDState());
  }
}