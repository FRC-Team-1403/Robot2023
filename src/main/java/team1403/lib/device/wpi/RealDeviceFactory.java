package team1403.lib.device.wpi;

import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.networktables.NetworkTableInstance;

import team1403.lib.device.AnalogDevice;
import team1403.lib.device.CougarAccelerometer;
import team1403.lib.device.CougarDoubleSolenoid;
import team1403.lib.device.DeviceFactory;
import team1403.lib.device.LimitSwitch;
import team1403.lib.device.MotorController;
import team1403.lib.device.PowerDistributor;
import team1403.lib.device.virtual.Limelight;
import team1403.lib.util.CougarLogger;


/**
 * A DeviceFactory that returns devices implemented using WPILibrary components.
 *
 * <p>These are real devices.
 */
@SuppressWarnings({"PMD.TooManyMethods"})
public class RealDeviceFactory implements DeviceFactory {
  /**
   * Returns a WpiBuiltinAccelerometer.
   */
  @Override
  public CougarAccelerometer makeBuiltinAccelerometer(
      String name, CougarAccelerometer.Range range) {
    return new WpiBuiltinAccelerometer(name, range);
  }

  /**
   * Returns a WpiPowerDistribution instance.
   */
  @Override
  public PowerDistributor makePowerDistributor(
      String name, CougarLogger logger) {
    return new WpiPowerDistribution(name, logger);
  }

  /**
   * Returns a Brushless CanSparkMax instance.
   */
  @Override
  public MotorController makeBrushlessCanSparkMax(
      String name, int channel, Type encoderType, CougarLogger logger) {
    return CougarSparkMax.makeBrushless(name, channel, encoderType, logger);
  }

  /**
   * Returns a Brushed CanSparkMax instance.
   */
  @Override
  public MotorController makeBrushedCanSparkMax(
      String name, int channel, Type encoderType, CougarLogger logger) {
    return CougarSparkMax.makeBrushed(name, channel, encoderType, logger);
  }

  /**
   * Returns a TalonSrx instance.
   */
  @Override
  public MotorController makeTalon(
      String name, int channel, CougarLogger logger) {
    return new TalonSrx(name, channel, logger);
  }

  /**
   * Returns a VictorSP instance.
   */
  @Override
  public MotorController makeVictorSpPwm(
      String name, int channel, CougarLogger logger) {
    return new VictorSp(name, channel, logger);
  }

  /**
   * Returns a VictorSpx instance.
   */
  @Override
  public MotorController makeVictorSpx(
      String name, int channel, CougarLogger logger) {
    return new VictorSpx(name, channel, logger);
  }

  /**
   * Returns a WpiLimitSwitch instance.
   */
  @Override
  public LimitSwitch makeLimitSwitch(String name, int channel) {
    return new WpiLimitSwitch(name, channel);
  }

  /**
   * Returns a WpiDoubleSolenoid instance.
   */
  @Override
  public CougarDoubleSolenoid makeDoubleSolenoid(
      String name, int forwardChannel,
      int reverseChannel, CougarLogger logger) {
    return new WpiDoubleSolenoid(name, forwardChannel, reverseChannel, logger);
  }

  /**
   * Returns a limelight instance.
   */
  public Limelight makeLimelight(String name) {
    return new Limelight(name, NetworkTableInstance.getDefault().getTable("limelight"));
  }

  /**
   * Returns a WpiAnalogDevice instance.
   */
  @Override
  public AnalogDevice makeAnalogDevice(String name, int channel) {
    return new WpiAnalogDevice(name, channel);
  }
}
