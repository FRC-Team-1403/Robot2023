package exceptions;

public class SubsystemNotFound extends RuntimeException {

    private static String blurb = "Unable to find subsystem - perhaps it was not added"
    + " to the initial Robot configuration in preview.Robot or the"
    + " provided name does not match the name inputted in the invokation of super() in the inheriting subsystem?";

    /**
     *
     */
    private static final long serialVersionUID = 1L;

    public SubsystemNotFound() {
        super(blurb);
    }

    public SubsystemNotFound(String subsystem) {
        super(blurb + " Offending subsystem: " + subsystem);
    }

}