package gui_elements;

import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.util.List;

import util.Pos;

/**
 * An interface for any GUI Elements that can be displayed or added as a child.
 * 
 * @author Brandon C.
 */
public interface Element {

    public static final int MARGIN = 10;

    /**
     * Sets the position of the Element and its children.
     * @param parentPos The position of this Element.
     */
    public default void update(Pos parentPos) {
        setPosition(parentPos.add(getRelativePosition()));
		for (int i = 0; i < getChildren().size(); i++) {
			Element element = getChildren().get(i);
			element.update(getPosition());
		}
    }

    /**
     * Sets the size based on the children to fit everything.
     */
    public default void resize() {
        Pos max = new Pos(getDefaultSize().x, getDefaultSize().y);
		for (int i = 0; i < getChildren().size(); i++) {
			Element element = getChildren().get(i);
			element.resize();
			element.update(getPosition());
			int currentX = element.getRelativePosition().x + element.getSize().x + MARGIN;
			int currentY = element.getRelativePosition().y + element.getSize().y + MARGIN; //bottom of the element
			if (max.y < currentY) {
				max.y = currentY;
			}
			if (max.x < currentX) {
				max.x = currentX;
			}
		}
		update(getPosition().subtract(getRelativePosition()));
		setSize(max);
    }

    /**
     * Called whenever the user clicks and returns the Element clicked on.
     * Calls processClick(e) on all children to determine if they were clicked.
     * @param e The mouse clicked event.
     * @return The element that was clicked on, out of the children and the Element.
     */
    public default Element processClick(MouseEvent e) {
        for (int i = 0; i < getChildren().size(); i++) {
			Element clicked = getChildren().get(i).processClick(e);
			if (clicked != null) {
				resize();
				return clicked;
			}
		}
		if (inBounds(e)) {
			return this;
		}
		return null;
    }

    /**
	 * Called when the mouse is moved. Calls process mouse move for its children.
	 * @param e The mouse moved event.
	 */
    public default void processMouseMove(MouseEvent e) {
        for (int i = 0; i < getChildren().size(); i++) {
			getChildren().get(i).processMouseMove(e);
		}
    }

    /**
     * Draws the Element and its children.
     */
    public void draw(Graphics g);

    /**
     * Gets the children of the Element.
     * @return list containing the children.
     */
    public List<Element> getChildren();

    /**
     * Adds children to the Element.
     * @param e The elements to be added.
     */
    public default void addChild(Element... e) {
        for (Element i : e) {
            getChildren().add(i);
		    i.resize();
            i.update(getPosition());
        }
    }

    /**
     * Removes children of the Element.
     * @param e The elements to remove.
     */
    public default void removeChild(Element... e) {
        for (Element i : e) {
            getChildren().remove(i);
            i.resize();
            i.update(getPosition());
        }
    }

    /**
     * Sets the relative position of this Element, relative to the position of its parent.
     * @param p The relative position.
     */
    public void setRelativePosition(Pos p);

    /**
     * Gets the relative position of this Element, relative to the position of its parent.
     * @return The relative position.
     */
    public Pos getRelativePosition();

    /**
     * Sets the absolute position of this Element
     */
    public void setPosition(Pos p);

    /**
     * Gets the absolute position of this Element
     * @return The absolute position.
     */
    public Pos getPosition();

    /**
     * Gets the size of the Element.
     * @return The size.
     */
    public Pos getSize();

    /***
     * Gets the default size of the Element.
     * @return The size.
     */
    public Pos getDefaultSize();

    /**
     * Sets the size of the Element.
     * @param p The size.
     */
    public void setSize(Pos p);

    /**
     * Gets whether the mouse is over the Element
     * @param e Mouse moved event
     * @return true if the mouse is in the Element's bounds
     */
    public boolean inBounds(MouseEvent e);
}