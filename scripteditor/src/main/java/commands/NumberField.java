package commands;

import java.awt.event.KeyEvent;
import java.util.Set;

import org.json.JSONObject;

import gui.MainWindow;
import gui_elements.TextField;

/**
 * A type of textfield that is used to input doubles as parameters for RobotCommands.
 * 
 * @author Brandon C.
 */
public class NumberField extends TextField implements Interactable<Double>{
	
	private static final Set<Character> numbers = Set.of('1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '.', '-');
	
	private double previous = 0.0;

	/**
	 * Create a new NumberField.
	 * @param name The name of the interactable.
	 */
	public NumberField(String name) {
		super(name, 0, 0, 100, 15);
		setText("0.0");
	}

	/**
	 * Rounds the value of the NumberField.
	 */
	private void round() {
		int i = getText().indexOf('.');
		if (i != -1 && i + 4 < getText().length()) {
			String cut = getText().substring(0, i + 4);
			double value = Double.parseDouble(cut);
			value = Math.round(value * 100) / 100.0;
			super.setText(Double.toString(value));
		} else {
			//super.setText(text);
		}
	}

	/**
	 * {@inheritDoc}
	 * Sets the text to empty if the value is 0.
	 */
	@Override
	public void onSelect() {
		previous = getValue();
		if (getValue() == 0.0) {
			setText("");
		}
		super.onSelect();
	}
	
	/**
	 * {@inheritDoc}
	 * Rounds the value of the NumberField. If the value was changed adds a new history entry.
	 */
	@Override
	public void onDeselect() {
		round();
		if (getValue() == 0.0) {
			setValue(0.0);
		}
		if (previous != getValue().doubleValue()) {
			MainWindow.getInstance().addHistoryEntry('n');
		}
		super.onDeselect();
	}

	/**
	 * Checks if the character makes a valid double when added to the text.
	 */
	@Override
	public boolean isValidCharacter(KeyEvent e) {
		for (char c : numbers) {
			if (e.getKeyCode() == c) {
				if (c == '.') {
					if (getText().indexOf('.') == -1) {
						return true;
					}
				} else if (c == '-') {
					if (getText().length() == 0) {
						return true;
					}
				} else {
					return true;
				}
			}
		}
		return false;
	}

	@Override
	public Double getValue() {
		String text = getText();
		if (text.length() == 0) {
			return 0.0;
		} else if (text.length() == 1 && (text.equals("-") || text.equals("."))) {
			return 0.0;
		} else {
			return Double.parseDouble(text);
		}
	}

	/**
	 * {@inheritDoc}
	 * Rounds the value.
	 */
	@Override
	public void setValue(Double d) {
		setText(Double.toString(d));
		round();
	}

	@Override
	public void setFromJson(JSONObject parameters) {
		setText(parameters.get(getName()).toString());
	}

	@Override
	public JSONObject addToJson(JSONObject parameters) {
		parameters.put(getName(), getValue());
		return parameters;
	}
}
