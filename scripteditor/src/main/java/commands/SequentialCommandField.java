package commands;

import org.json.JSONObject;

import gui.MainWindow;

import java.awt.event.MouseEvent;

import gui_elements.Button;
import gui_elements.Element;
import util.Pos;

public class SequentialCommandField extends Button implements Interactable<RobotCommand>{

    private static final int MARGIN = 10;
	private static final int DEFAULT_WIDTH = 200;
	private static final int DEFAULT_HEIGHT = 50;
    private static final int END_HEIGHT = 30;

    private RobotCommand command;
    private String name;

    public SequentialCommandField() {
        super("", 0, 0, DEFAULT_WIDTH, DEFAULT_HEIGHT);
    }

    public boolean hasCommand()
    {
        return this.command != null;
    }

    public void removeCommand()
    {
        if(hasCommand())
        {
            removeChild(command);
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
		JSONObject commandJSON = parameters.getJSONObject("Command");
		setValue(RobotCommand.getFromJSON(commandJSON));
    }

    @Override
    public JSONObject addToJson(JSONObject parameters) {
		if (command == null) {
			return null;
		}
		parameters.put("Command", command.getJSON());
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

    @Override
    public RobotCommand getValue() {
        return command;
    }

    @Override
    public void setValue(RobotCommand c) {
        if (hasCommand()) {
			removeCommand();
		}
		if (c == null) {
			return;
		}
		this.command = c;
		c.setRelativePosition(new Pos(MARGIN, MARGIN + END_HEIGHT));
        addChild(c);
    }

}