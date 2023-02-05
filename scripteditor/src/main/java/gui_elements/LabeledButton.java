package gui_elements;

import java.awt.Graphics;

import util.Pos;

/**
 * A button which has a name that displays to the left of the Button.
 * The position of the button is offset by the width of the name.
 * 
 * @author Brandon C.
 */
public class LabeledButton extends Button{

    private String name;
    private int stringWidth = 0;

    /**
     * Create a new labeled button.
     * @param name The name/label of the button.
     * @param text The text to display in the button.
     * @param x The x position.
     * @param y The y position.
     * @param w The width.
     * @param h The height.
     */
    public LabeledButton(String name, String text, int x, int y, int w, int h) {
        super(text, x, y, w, h);
        this.name = name;
    }

    /**
     * Gets the name/label of the labeled button.
     * @return The name/label of the button.
     */
    public String getName() {
        return name;
    }

    /**
     * Sets the name/label of the labeled button.
     * @param name The name to use.
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * {@inheritDoc}
     * The button is offset by the width of the label.
     */
    @Override
    public void draw(Graphics g) {
		boolean changed = false;
		if (stringWidth == 0) {
			changed = true;
		}
		stringWidth = g.getFontMetrics().stringWidth(getName()) + 5;
		if (changed) {
			update(new Pos(x - getRelativePosition().x, y - getRelativePosition().y));
		}
        g.drawString(getName(), x-stringWidth, y + 12);
        super.draw(g);
    }

    /**
     * {@inheritDoc}
     * The button is offset by the width of the label.
     */
    @Override
	public void update(Pos parentPos) {
		super.update(new Pos(parentPos.x + stringWidth, parentPos.y));
	}

}