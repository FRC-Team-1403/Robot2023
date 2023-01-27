package team1403.lib.device;

import org.photonvision.PhotonCamera;


import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends PhotonCamera {

    public Limelight(String cameraName) {
        super(NetworkTableInstance.getDefault(), cameraName);
    }
    
}