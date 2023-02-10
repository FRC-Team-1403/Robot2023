package commands;

import java.awt.Color;
import java.awt.Graphics;

import org.json.JSONObject;

import gui.MainWindow;
import gui_elements.Button;
import gui_elements.EventButton;
import util.Point;
import util.Pos;

/**
 * An interactable that stores a Point.
 * The PointSelect has 2 NumberFields that can be used to set the x and y of the point.
 * It also has a button that can be used to select a point by clicking on other point selects (copy point) or the field image.
 * 
 * @author Brandon C.
 */
public class PointSelect extends Button implements Interactable<Point>{

    public static final Color HIGHLIGHT_COLOR = new Color(255, 248, 54);
    public static final Color DEFAULT_COLOR = new Color(130, 255, 239);
    
    private static final Pos SELECT_POS = new Pos(130, 20);
    private static final int MARGIN = 10;
    
    private final NumberField xField, yField;
    private final EventButton select;
    private PointSelect next;
    
    /**
     * Create a new PointSelect with a name.
     * @param name The name of the interactable.
     */
    public PointSelect(String name) {
        this();
        setName(name);
    }

    /**
     * Create a new PointSelect.
     */
    public PointSelect() {
        super("", 0, 0, 220, 60);
        setColor(DEFAULT_COLOR);
        xField = new NumberField("x");
        xField.setRelativePosition(new Pos(MARGIN, 20));
        yField = new NumberField("y");
        yField.setRelativePosition(new Pos(MARGIN, 20 + 20));
        PointSelect ps = this;
        select = new EventButton("Select Point", SELECT_POS.x, SELECT_POS.y, 80, 35, true) {
			@Override
			public void onClick() {
                MainWindow.getInstance().enterPointSelection(ps);
			}
        };
        addChild(xField, yField, select);
    }
    
    /**
     * Gets the next point select to use after the current one (automatic if in a list).
     * @return The next PointSelect.
     */
    public PointSelect getNext() {
        return next;
    }

    /**
     * Sets the next point select to use after the current one.
     */
    public void setNext(PointSelect next) {
        this.next = next;
    }

    /**
     * {@inheritDoc}
     * If the PointSelect is hovered it will draw a circle around its point on the preview.
     */
    @Override
    public void draw(Graphics g) {
        super.draw(g);
        if (getHovered()) {
            Point p = getValue().toPixels();
            g.setColor(Color.blue);
            g.drawOval((int) p.x - 10, (int) p.y - 10, 20, 20);
        }
    }

    @Override
    public void setValue(Point p) {
        xField.setValue(p.x);
        yField.setValue(p.y);
    }

    @Override
    public Point getValue() {
        return new Point(xField.getValue(), yField.getValue());
    }

    @Override
    public void setFromJson(JSONObject json) {
        setValue(new Point(json.getDouble("x"), json.getDouble("y")));
    }

    @Override
    public JSONObject addToJson(JSONObject parameters) {
        parameters.put("x", xField.getValue());
        parameters.put("y", yField.getValue());
		return parameters;
	}

    @Override
    public String getName() {
        return getText();
    }

    @Override
    public void setName(String name) {
        setText(name);
    }    
}