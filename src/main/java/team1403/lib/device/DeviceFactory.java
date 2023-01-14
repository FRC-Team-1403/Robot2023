package team1403.lib.device;

import com.revrobotics.SparkMaxRelativeEncoder;

import team1403.lib.util.CougarLogger;

/**
 * Creates devices.
 *
 * <p>Subsystems create their devices through this factory.
 */
public interface DeviceFactory {

  /**
   * Creates a PowerDistributor for monitoring the power panel.
   *
   * <p>This is implicitly the default distributor. In practice we
   * only have one so this is assumed to be the standard CAN bus wiring.
   *
   * @param name The name to give the distributor.
   * @param logger The logger to give the device.
   *
   * @return new PowerDistributor.
   */
  public PowerDistributor makePowerDistributor(String name, CougarLogger logger);

  /**
   * Create an accelerometer from the RoboRIO.
   *
   * @param name The name for the accelerometer.
   * @param range The max sensitivity.
   *
   * @return accelerometer for the RoboRIO.
   */
  public CougarAccelerometer makeBuiltinAccelerometer(
      String name, CougarAccelerometer.Range range);

  /**
   * Creates a Brushed CANSparkMax MotorController.
   *
   * @param name The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   * @param encoderType The type of encoder used with the motor controller
   * @param logger The logger to use with the new instance.
   *
   * @return a new MotorController for a CANSparkMax.
   */
  public MotorController makeBrushedCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger);

  /**
   * Creates a Brusheless CANSparkMax MotorController.
   *
   * @param name The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   * @param encoderType The type of encoder used with the motor controller
   * @param logger The logger to use with the new instance.
   *
   * @return a new MotorController for a CANSparkMax.
   */
  public MotorController makeBrushlessCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType, CougarLogger logger);
  
  /**
   * Creates a TalonSrx MotorController.
   *
   * @param name The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   * @param logger The logger to use with the new instance.
   *
   * @return a new MotorController for a TalonSRX.
   */
  public MotorController makeTalon(
      String name, int channel, CougarLogger logger);

  /**
   * Creates a VictorSpPwm MotorController.
   *
   * @param name The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   * @param logger The logger to use with the new instance.
   *
   * @return a new MotorController for a VictorSp.
   */
  public MotorController makeVictorSpPwm(
      String name, int channel, CougarLogger logger);

  /**
   * Creates a VictorSpx MotorController.
   *
   * @param name The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   * @param logger The logger to use with the new instance.
   *
   * @return a new MotorController for a VictorSpx.
   */
  public MotorController makeVictorSpx(
      String name, int channel, CougarLogger logger);

  /**
   * Creates a limit switch.
   *
   * @param name The name of the new device instance.
   * @param channel The robotRIO port the limit switch is on.
   *
   * @return a new LimitSwitch for a WPILib LimitSwitch.
   */
  public LimitSwitch makeLimitSwitch(String name, int channel);

  /**
   * Creates a double solenoid.
   *
   * @param name The name of the new device instance.
   * @param forwardChannel The port in which the forwardChannel is being controlled.
   * @param reverseChannel The port in which the reverseChannel is being controlled.
   * @param logger The logger to use with the new instance.
   * 
   * @return a new DoubleSolenoid for a WPILib DoubleSolenoid
   */
  public CougarDoubleSolenoid makeDoubleSolenoid(String name, 
                              int forwardChannel, int reverseChannel, CougarLogger logger);

  /**
   * Creates an analog device.
   *
   * @param name The name of the new device instance.
   * @param channel The robotRIO port the analog device is on.
   *
   * @return a new AnalogDevice for a WPILib AnalogDevice.
   */
  public AnalogDevice makeAnalogDevice(String name, int channel);
}
