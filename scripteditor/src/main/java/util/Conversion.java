package util;

public class Conversion {
    public static double inchesToMeters(double inch) {
        final double conversionNum =  39.37;
        if (inch == 0.0){
            return 0.0;
        }
        return inch / conversionNum;
    }
}
