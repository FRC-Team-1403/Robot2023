package team1403.lib.device;

import edu.wpi.first.wpilibj.Compressor;;

public interface CougarCompressor extends Actuator {

    /**
     * Starts the Compressor.
     * 
     * @param type The type of mode.
     */
    void start(String type);

    /**
     * Stops the Compressor.
     */
    void stop();

    /**
     * Gets the Pressure Switch Value.
     *
     * @return The pressure switch value
     */
    double getPressureSwitchValue();

    /**
     * Sets the pressure value of the Compressor.
     *
     * @param val The new pressure value.
     */
    void setPressure(int val);

    /**
     * Checks if the Compressor is enabled. 
     *
     * @return The status of the Compressor.
     */
    boolean isEnabled();

    /**
     * Returns the current flowing through the compressor motor.
     * @return the current value
     */
    double getCurrent();

    /**
     * Returns a boolean indicating whether the compressor is in closed-loop control mode.
     *
     * @return The boolean value corressponding to the Compressor's mode.
     */
    boolean getClosedLoopControl();

    /**
     * Sets the compressor to closed-loop control mode if value is true, or open-loop control mode if value is false.
     *
     * @param value The status of the loop control mode.
     */
    void setClosedLoopControl(boolean value);

    /**
     * returns a boolean indicating whether the compressor has detected a fault due to excessive current.
     *
     * @return The boolean indictating any fault in excessive current.
     */
    boolean getCompressorCurrentTooHighStickyFault();

    /**
     * returns a boolean indicating whether the compressor has detected a sticky fault due to a shorted motor.
     * @return The boolean indictating any sticky fault due to a shorted motor.
     */
    boolean getCompressorShortedStickyFault();

    /**
     * Returns a boolean indicating whether the compressor has detected a sticky fault due to a disconnected motor.
     *
     * @return The boolean indictating any sticky fault due to a disconnected motor.
     */
    boolean getCompressorNotConnectedStickyFault();

    /**
     * Clears all the Sticky faults.
     */
    void clearAllPCMStickyFaults();



}
