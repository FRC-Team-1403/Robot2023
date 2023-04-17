
package team1403.robot.chargedup.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.robot.chargedup.StateManager;
import team1403.robot.chargedup.StateManager.GamePiece;

public class PhotonVisionSubsystem extends CougarSubsystem {
  private PhotonCamera limeLight;
  private PhotonPipelineResult result;

  private int pipelineIndex;

  private double cameraHeightMeters = 1;
  private double targetHeightMeters = 1;
  private double cameraPitchDegrees = -30;

  public PhotonVisionSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("Vision Subsystem", injectedParameters);
    // Photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);
    limeLight = new PhotonCamera("OV5647");

    // 0: April Tags
    // 1: Reflective Tape
    limeLight.setPipelineIndex(0);
    
    result = new PhotonPipelineResult();

    //Cone detection
  }

  public double getHorizontalOffsetOfTarget() {
    result = limeLight.getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getYaw() : 0;
  }

  public double getDistanceFromTarget() {
    result = limeLight.getLatestResult();
    if (result.hasTargets()) {
      double distanceToTarget =  PhotonUtils.calculateDistanceToTargetMeters(
        cameraHeightMeters,
        targetHeightMeters,
        Units.degreesToRadians(cameraPitchDegrees),
        Units.degreesToRadians(result.getBestTarget().getPitch()));
      return distanceToTarget;
    }
    return 0;
  }

  @Override
  public void periodic() {
    pipelineIndex = StateManager.getInstance().getGamePiece() == GamePiece.CUBE ? 0 : 1;
    limeLight.setPipelineIndex(pipelineIndex);
  }
}
