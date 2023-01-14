package team1403.robot.__replaceme__;

import java.util.function.Function;

import edu.wpi.first.wpilibj.RobotBase;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.WpiLibRobotAdapter;

/**
 * It is unlikely that anything will go here.
 *
 * <p>This is only a wrapper to bootstrap the WPI libraries.
 * The WPI library will construct our CougarRobot class and call
 * into it.
 *
 * <p>Our code is in the CougarRobotImpl class and its dependencies.
 */
public final class Main {
  /**
   * Class is not instantiatable.
   */
  private Main() {}

  /**
   * Robot program entry point.
   *
   * @param args Command line args from java execution.
   */
  public static void main(String... args) {
    // This is going to create our CougarRobotImpl when called.
    Function<CougarLibInjectedParameters, CougarRobotImpl> cougarFactory =
        (CougarLibInjectedParameters params) -> {
          RobotConfig config = new RobotConfig();
          return new CougarRobotImpl(params, config);
        };

    RobotBase.startRobot(
        () -> {
          return new WpiLibRobotAdapter<CougarRobotImpl>(cougarFactory);
        });
  }
}
