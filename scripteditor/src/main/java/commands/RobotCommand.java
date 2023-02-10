package commands;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Toolkit;
import java.awt.datatransfer.StringSelection;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.json.JSONException;
import org.json.JSONObject;

import gui_elements.Button;
import gui_elements.EventButton;
import preview.Robot;
import util.Pos;

/**
 * A class to represent a command given to the robot in a visual format.
 * The RobotCommand contains a list of interactables which can be used to input values
 * through the GUI.
 * It also features the ability to export all the values of its interactables as a JSONObject,
 * and load them from one as well.
 * 
 * @author Brandon C.
 */
@SuppressWarnings("rawtypes")
public abstract class RobotCommand extends Button{

	private static final int TITLE_SPACE = 30;
	private static final int MARGIN = 10;
	private static final int COPY_WIDTH = 50;
	private static final int COPY_HEIGHT = 15;

	/**
	 * The internal list of interactables that the robotcommand will use when exporting to a JSON format.
	 */
	private final List<Interactable> interactables;
	private final EventButton copyButton;
	private Robot initial;
	private Robot output;

	/**
	 * Creates a new RobotCommand.
	 * @param name The name of the command.
	 * @param color The background color of the command.
	 * @param fieldNames The names of the double fields for the command. These will be automatically created.
	 * @param buttonNames The names of the boolean buttons for hte command. These will be automatically created.
	 */
	public RobotCommand(String name, Color color, Interactable... interactables) {
		super(name, 0, 0, 200, TITLE_SPACE);
		setColor(color);

		this.interactables = Arrays.asList(interactables);
		int currentY = TITLE_SPACE;
		copyButton = new EventButton("Copy", 0, 0, COPY_WIDTH, COPY_HEIGHT, true) {
			@Override
			public void onClick() {
				Toolkit.getDefaultToolkit().getSystemClipboard().setContents(new StringSelection(getJSON().toString()), null);
			}
		};
		addChild(copyButton);
		for (int i = 0; i < interactables.length; i++) {
			Interactable a = interactables[i];
			addChild(a);
			a.setRelativePosition(new Pos(MARGIN, currentY));
			currentY += a.getSize().y + MARGIN;
		}
		resize();
	}

	/**
	 * Gets the interactable with the given name.
	 * @param name The name of the interactable.
	 * @return Interactable with the corresponding name. Null if there are no interactables with the name.
	 */
	public Interactable getInteractable(String name) {
		for (int i = 0; i < interactables.size(); i++) {
			if (interactables.get(i).getName().equals(name)) {
				return interactables.get(i);
			}
		}
		return null;
	}

	/**
	 * Gets the double input field from its name.
	 * @param name The name of the field.
	 * @return The input field object with the given name.
	 */
	public NumberField getNumberFromName(String name) {
		Interactable i = getInteractable(name);
		if (i instanceof NumberField) {
			return (NumberField) i;
		}
		return null;
	}
	
	/**
	 * Gets the boolean button from its name.
	 * @param name The name of the button.
	 * @return The input button object with the given name.
	 */
	public ToggleButton getBooleanFromName(String name) {
		Interactable i = getInteractable(name);
		if (i instanceof ToggleButton) {
			return (ToggleButton) i;
		}
		return null;
	}

	/**
	 * Gets the values represented by the interactables of the RobotCommand in the form of a JSONObject.
	 * @return A JSONObject containing the information of the interactables in the RobotCommand.
	 */
	public JSONObject getJSON() {
		JSONObject command = new JSONObject();
		JSONObject parameters = new JSONObject();
		for (int i = 0; i < interactables.size(); i++) {
			interactables.get(i).addToJson(parameters);
		}
		command.put("CommandName", getText());
		command.put("Parameters", parameters);
		return command;
	}

	/**
	 * Sets the initial and output robots based on the applyCommand method.
	 * @param initial The initial state to use.
	 */
	public void updateRobot(Robot initial) {
		this.initial = initial;
		this.output = applyCommand(initial);
	}

	/**
	 * Draws the preview of the RobotCommand on the field image.
	 * @param g The graphics object to use.
	 */
	public void drawPreview(Graphics g) {
		g.setColor(Color.red);
		initial.getPosition().toPixels().draw(g);
		output.getPosition().toPixels().draw(g);
		if (getHovered()) {
			initial.draw(g);
			output.draw(g);
		}
	}

	/**
	 * Gets the Robot that represents before the command is applied.
	 * @return initial robot.
	 */
	public Robot getInitial() {
		return initial;
	}

	/**
	 * Gets the Robot that represents after the command is applied.
	 * @return output robot.
	 */
	public Robot getOutput() {
		return output;
	}

	/**
	 * Applies the effect of the RobotCommand on the Robot representation.
	 * Example: a drive forward command would take the initial position of the robot and return a copy
	 * that is translated by the amount specified in the "distance" double field.
	 * @param g The graphics object to use to draw.
	 * @param initial The state of the robot representation before the command's effect is accounted for
	 * @return The state of the robot representation after the command's effect is accounted for
	 */
	public abstract Robot applyCommand(Robot initial);

	/**
	 * {@inheritDoc}
	 * Handles the copy button.
	 */
	@Override
	public void resize() {
		removeChild(copyButton);
		super.resize();
		addChild(copyButton);
		copyButton.setRelativePosition(new Pos(w - COPY_WIDTH - MARGIN, MARGIN));
	}

	/**
	 * Creates a RobotCommand from the JSON formatted information.
	 * @param commandJSON The JSONObject that stores the information to create the RobotCommand from.
	 * @return A RobotCommand with the information from the JSONObject.
	 */
	public static RobotCommand getFromJSON(JSONObject commandJSON) {
		String commandName = (String) commandJSON.get("CommandName");
		JSONObject parameters = (JSONObject) commandJSON.get("Parameters");
		Supplier<RobotCommand> supplier = Commands.commandMap.get(commandName);
		if (supplier != null) {
			RobotCommand c = supplier.get();
			for (int i = 0; i < c.interactables.size(); i++) {
				try {
					c.interactables.get(i).setFromJson(parameters);
					c.resize();
				} catch (JSONException e) {
					e.printStackTrace();
				}
			}
			return c;
		}
		return null;
	}
}
