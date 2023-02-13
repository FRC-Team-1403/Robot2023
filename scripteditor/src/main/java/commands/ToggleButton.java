package commands;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;

import org.json.JSONObject;

import gui.MainWindow;
import gui_elements.Element;
import gui_elements.LabeledButton;

/**
 * An interactable that stores a boolean.
 * When clicked, the boolean toggles.
 * Green = true, red = false
 * 
 * @author Brandon C.
 */
public class ToggleButton extends LabeledButton implements Interactable<Boolean>{

	private static final Color TRUE_COLOR = new Color(163, 252, 139);
	private static final Color FALSE_COLOR = new Color(245, 66, 66);

	private boolean value;

	/**
	 * Create a new Toggle Button.
	 * @param name The name of the interactable.
	 */
	public ToggleButton(String name) {
		super(name, "", 0, 0, 20, 20);
	}
	
	/**
	 * Draws a green button for true and a red button for false.
	 */
	@Override
	public void draw(Graphics g) {
		if (value) {
			setColor(TRUE_COLOR);
		} else {
			setColor(FALSE_COLOR);
		}
		super.draw(g);
	}

	/**
	 * {@inheritDoc}
	 * Toggles the value of the button.
	 */
	@Override
	public Element processClick(MouseEvent e) {
		if (inBounds(e)) {
			value = !value;
			MainWindow.getInstance().addHistoryEntry('b');
		}
		return super.processClick(e);
	}

	@Override
	public void setFromJson(JSONObject parameters) {
		value = parameters.getBoolean(getName());
	}
	
	@Override
	public JSONObject addToJson(JSONObject parameters) {
		parameters.put(getName(), value);
		return parameters;
	}

	@Override
	public Boolean getValue() {
		return value;
	}

	@Override
	public void setValue(Boolean value) {
		this.value = value;
	}
}
