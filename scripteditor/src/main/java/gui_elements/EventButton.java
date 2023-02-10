package gui_elements;

import java.awt.event.MouseEvent;

/**
 * A button that has an onClick() method.
 * Calls the onClick() method when clicked.
 * 
 * @author Brandon C.
 */
public abstract class EventButton extends Button{
	
	/**
	 * Create a new EventButton.
	 * @param text The text in the EventButton.
	 * @param x The x position.
	 * @param y The y position.
	 * @param w	The width.
	 * @param h The height.
	 * @param centered Whether the text is centered or not.
	 */
	public EventButton(String text, int x, int y, int w, int h, boolean centered) {
		super(text, x, y, w, h);
		setCentered(centered);
	}
	
	/**
	 * {@inheritDoc}
	 * Calls onClick() if the button was clicked.
	 */
	@Override
	public Element processClick(MouseEvent e) {
		if (inBounds(e)) {
			onClick();
		}
		return super.processClick(e);
	}

	/**
	 * Called when the EventButton is clicked.
	 */
	public abstract void onClick();
}
