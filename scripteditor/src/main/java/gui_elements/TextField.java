package gui_elements;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;

/**
 * A labeled button that allows the user to input text into the button area.
 * 
 * @author Brandon C.
 */
public class TextField extends LabeledButton{
	
	private static final Color selectedColor1 = new Color(181, 209, 255);
	private static final Color selectedColor2 = new Color(84, 149, 255);
	
	private static TextField selectedTextField;
	private int counter = 0;
	private boolean selected = false;
	
	/**
	 * Create a new TextField.
	 * @param name The name of the text field.
	 * @param x The x position.
	 * @param y The y position.
	 * @param w The width.
	 * @param h The height.
	 */
	public TextField(String name, int x, int y, int w, int h) {
		super(name, "", x, y, w ,h);
		setText("");
	}

	/**
	 * Called when the text field is selected.
	 */
	public void onSelect() {
		selected = true;
		counter = 50;
	}
	
	/**
	 * Called when the text field is deselected.
	 */
	public void onDeselect() {
		selected = false;
		counter = 0;
		setColor(Button.DEFAULT_BACKGROUND_COLOR);
	}
	
	/**
	 * Adds a character to the text of the textfield.
	 */
	public void addCharacter(char c) {
		setText(getText() + c);
	}
	
	/**
	 * Updates the text of the Text Field based on the Key Pressed event.
	 * If the KeyEvent is a valid character it will add the character to the text.
	 * If the KeyEvent is backspace it will remove one from the end of the text.
	 * @param e The key pressed event.
	 */
	public void processKeyEvent(KeyEvent e) {
		if (e.getKeyCode() == KeyEvent.VK_BACK_SPACE && getText().length() >= 1) {
			setText(getText().substring(0, getText().length()-1));
		} else {
			if (isValidCharacter(e)) {//check for actual letter
				addCharacter(e.getKeyChar());
			}
		}
	}

	/**
	 * Determines if a character is valid to be added to the text.
	 * @param e The key pressed event.
	 * @return Whether the character is valid or not.
	 */
	public boolean isValidCharacter(KeyEvent e) {
		return KeyEvent.getKeyText(e.getKeyCode()).length() == 1 || e.getKeyCode() == KeyEvent.VK_SPACE;
	}

	/**
	 * Deselects the currently selected text field.
	 */
	public static void deselectTextField() {
		if (selectedTextField != null) {
			selectedTextField.onDeselect();
		}
		selectedTextField = null;
	}

	/**
	 * Selects a text field.
	 * @param tf the text field to select.
	 */
	public static void selectTextField(TextField tf) {
		deselectTextField();
		selectedTextField = tf;
		tf.onSelect();
	}

	/**
	 * Gets the currently selected text field.
	 * @return The currently selected text field.
	 */
	public static TextField getSelectedTextField() {
		return selectedTextField;
	}

	/**
	 * {@inheritDoc}
	 * Cycles between the select colors if selected.
	 */
	@Override
	public void draw(Graphics g) {
		if (selected) {
			counter++;
			counter %= 100;
			if (counter < 50) {
				setColor(selectedColor1);
			} else {
				setColor(selectedColor2);
			}
		}
		super.draw(g);
	}
	
	/**
	 * {@inheritDoc}
	 * Selects the text field.
	 */
	@Override
	public Element processClick(MouseEvent e) {
		if (inBounds(e)) {
			selectTextField(this);
		}
		return super.processClick(e);
	}
}
