package commands;

import java.awt.event.MouseEvent;

import org.json.JSONObject;

import gui.MainWindow;
import gui_elements.Button;
import gui_elements.Element;
import util.Pos;

/**
 * An interactable used by the ParallelCommand that stores a command.
 * Also contains a toggleButton that tells whetehr the command is an end command or not.
 * 
 * @author Brandon C.
 */
public class ParallelCommandField extends Button implements Interactable<RobotCommand>{
	
	private static final int MARGIN = 10;
	private static final int DEFAULT_WIDTH = 200;
	private static final int DEFAULT_HEIGHT = 50;
	private static final int END_HEIGHT = 30;

	private String name;
	private RobotCommand command;
	private ToggleButton endCondition;
	
	/**
	 * Creates a new ParallelCommandField.
	 */
	public ParallelCommandField () {
		super("", 0, 0, DEFAULT_WIDTH, DEFAULT_HEIGHT);
		endCondition = new ToggleButton("End Condition");
		endCondition.setRelativePosition(new Pos(MARGIN, 2 * MARGIN));
	}

	/**
	 * Determines whether the field has a command.
	 * @return true if command is not null.
	 */
	public boolean hasCommand() {
		return this.command != null;
	}

	/**
	 * Sets the current command to null and removes it from the children.
	 * Also removes the button temporarily.
	 */
	public void removeCommand() {
		if (hasCommand()) {
			removeChild(command, endCondition);
			command.setRelativePosition(new Pos(0, 0));
			command = null;
		}
	}

	/**
	 * {@inheritDoc}
	 * Interacts with the MainWindow to set the command based on the MouseCommand.
	 * If the field is clicked while there is no MouseCommand, it will pick the command out from the field.
	 * If the field is clicked while there is a MouseCommand it will swap with the current command.
	 * Addsa history entry.
	 */
	@Override
	public Element processClick(MouseEvent e) {
		Element clicked = super.processClick(e);
		if (inBounds(e)) {
			MainWindow instance = MainWindow.getInstance();
			RobotCommand mouseCommand = instance.getMouseCommand();
			if (mouseCommand == null && hasCommand() && clicked == command) {
				instance.setMouseCommand(getValue());
				removeCommand();
				instance.addHistoryEntry('p');
				return this;
			} else {
				if (!hasCommand() || clicked == command) {
					instance.setMouseCommand(command);
					removeCommand();
					setValue(mouseCommand);
					if (mouseCommand != null) {
						instance.addHistoryEntry('p');
					}
					return this;
				}
			}
		}
		return clicked;
	}

	@Override
	public void setFromJson(JSONObject parameters) {
		endCondition.setValue(parameters.getBoolean("EndCondition"));
		JSONObject commandJSON = parameters.getJSONObject("Command");
		setValue(RobotCommand.getFromJSON(commandJSON));
	}

	@Override
	public JSONObject addToJson(JSONObject parameters) {
		if (command == null) {
			return null;
		}
		parameters.put("Command", command.getJSON());
		parameters.put("EndCondition", endCondition.getValue());
		return parameters;
	}

	@Override
	public String getName() {
		return name;
	}
	
	@Override
	public void setName(String name) {
		this.name = name;
		setText(name);
	}
	
	/**
	 * {@inheritDoc}
	 * If the field currently has a command it will remove it as a child.
	 * Sets the relative position of the command in the field and adds it as a child.
	 */
	@Override
	public void setValue(RobotCommand c) {
		if (hasCommand()) {
			removeCommand();
		}
		if (c == null) {
			return;
		}
		this.command = c;
		c.setRelativePosition(new Pos(MARGIN, 2 * MARGIN + END_HEIGHT));
		addChild(c, endCondition);
	}

	@Override
	public RobotCommand getValue() {
		return this.command;
	}
}
