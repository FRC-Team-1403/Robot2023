package team1403.lib.subsystems;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.SwerveModule;
/**
 * The SwerveSubsytem contains the 4 different SwerveModules used in the robot.
 */
public class SwerveSubsystem extends CougarSubsystem {
  
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  public SwerveSubsystem(String name, CougarLibInjectedParameters injectedParameters) {
    super(name, injectedParameters);

  }

}
