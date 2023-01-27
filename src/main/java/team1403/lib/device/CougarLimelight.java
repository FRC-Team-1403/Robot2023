package team1403.lib.device;

public abstract class CougarLimelight implements Sensor {
    
    private final double targetHeight;//inches
    private final double cameraHeight;//inches
    private final double cameraAngle;//degrees
    private final double zOffset;
    private final double errorLeniency;
    private boolean shouldAdjust;

    public CougarLimelight(String name, double targetHeight, double cameraHeight, double cameraAngle, double zOffset, double errorLeniency) {
        this.targetHeight = targetHeight;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.zOffset = zOffset;
        this.errorLeniency = errorLeniency;
        shouldAdjust = false;
    }
    
    public void configureUseMode(boolean adjust){
        this.shouldAdjust = adjust;
    }

    public boolean getShouldAdjust(){
        return shouldAdjust;
    }

    public abstract void turnOn();

    public abstract void turnOff();
    
    public abstract double getX();
 
    public abstract double getY();

    public abstract double getV();

    public double getAdjustedX(double fieldRelativeAngle){
        double phi = 90 - getX() - fieldRelativeAngle;
        double dist = getDistanceToTarget();
        double comp = phi - Math.atan2((dist*Math.sin(Math.toRadians(phi))), dist * Math.cos(Math.toRadians(phi)) + getZOffset());
        comp = Math.toDegrees(comp);
        //SmartDashboard.putNumber("Comp", comp);
        return (comp + getX());
    };

    public boolean isCentered(){
        return Math.abs(getX()) < getErrorLeniency();
    };

    public boolean hasTarget(){
        if(getV() == 0) {
            return false;
        }
        return true;
    };

    public double getDistanceToTarget(){
        //subtracting getEntry("ty") because limelight is upside down
        return (getTargetHeight() - getCameraHeight()) 
            / Math.tan((getCameraAngle() - getY()) / 180 * Math.PI);
    };

    public double getTargetHeight(){
        return targetHeight;
    }

    public double getCameraHeight(){
        return cameraHeight;
    }

    public double getCameraAngle(){
        return cameraAngle;
    }

    public double getZOffset(){
        return zOffset;
    }

    public double getErrorLeniency(){
        return errorLeniency;
    }


}
