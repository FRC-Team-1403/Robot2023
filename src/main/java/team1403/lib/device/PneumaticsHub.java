package team1403.lib.device;

/**
 * Represents the Pneumatics Hub.
 */
public interface PneumaticsHub extends CougarDoubleSolenoid{

    /**
    * Returns the current voltage being drawn by the distributor.
    *
    * @return volts
    */
    double getVoltage();
    







}