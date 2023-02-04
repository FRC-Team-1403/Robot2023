package team1403.lib.util;

public class Dimension {
    private final double length;
    private final double width;
    private final double depth;

    public Dimension(double height, double width) {
        this.length = height;
        this.width = width;
        this.depth = 0;
    }

    public Dimension(double height, double width, double depth) {
        this.length = height;
        this.width = width;
        this.depth = depth;
    }

    public double getLength() {
        return length;
    }

    public double getWidth() {
        return width;
    }

    public double getDepth() {
        return depth;
    }

}
