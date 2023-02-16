// package arm;

// import org.junit.jupiter.api.Test;

// import team1403.lib.core.CougarLibInjectedParameters;
// import team1403.lib.device.test.FakeCurrentSensor;
// import team1403.lib.device.test.FakeEncoder;
// import team1403.lib.device.test.FakeMotorController;
// import team1403.lib.device.test.MappedDeviceFactory;
// import team1403.lib.device.virtual.ManualLimitSwitch;
// import team1403.lib.util.CougarLogger;
// import team1403.robot.chargedup.RobotConfig;
// import team1403.robot.chargedup.arm.Arm;

// /**
//  * Unit test for checking the bounds for the arm.
//  */
// public class ArmBoundsTest {
  
//   @Test
//   void testConstructor() {
//     fakeParts();
//     var parameters = fakeParts().deviceFactory(fakeParts())
//         .build();
//   }

//   /**
//    * Method for defining the fake parts for testing.
//    */
//   public void fakeParts() {
//     frontLimitSwitch = new ManualLimitSwitch("Arm.Fron");
//     telescopicLimitSwitch = new ManualLimitSwitch("Arm.Telescopic");
//     fakeCurrentSensor = new FakeCurrentSensor("Arm.CurrentSensor");
//     fakeEncoder = new FakeEncoder("Rail.Encoder", kticksPerRevolution);
//     fakeMotor = new FakeMotorController("Rail.Motor", logger, fakeEncoder, fakeCurrentSensor);
//   }

//   public CougarLogger logger = CougarLogger.getCougarLogger("Arm");
//   public ManualLimitSwitch frontLimitSwitch;
//   public ManualLimitSwitch telescopicLimitSwitch;
//   public FakeCurrentSensor fakeCurrentSensor;
//   public FakeEncoder fakeEncoder;
//   public FakeMotorController fakeMotor;
//   public MappedDeviceFactory deviceFactory;

//   public static final double kticksPerRevolution = 1024.0;
// }