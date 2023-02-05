package gui;

import java.awt.Color;
import java.awt.FontMetrics;
import java.awt.Toolkit;
import java.awt.datatransfer.DataFlavor;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import commands.Commands;
import commands.PointSelect;
import commands.RobotCommand;
import gui_elements.Button;
import gui_elements.Element;
import gui_elements.EventButton;
import gui_elements.TextField;
import preview.Robot;
import util.Point;
import util.Pos;

/**
 * The Main Window that displays the GUI.
 * 
 * @author Brandon C.
 */
public class MainWindow extends Window implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener {
	
	private static final Color insertColor = new Color(163, 252, 139);
	private static final Color backgroundColor = new Color(235, 235, 235);
	private static final Color REMOVE_COLOR = new Color(255, 112, 102);
	private static MainWindow instance = null;
	
	private FilePicker picker = new FilePicker();
	private Pos m = new Pos(0,0);
	private MouseEvent mEvent;
	private Pos prevM = new Pos(0,0);
	private final int margin = 10;
	private final Button qRect = new Button("", margin, 200, 455, 1000);
	private final Button buttonRect = new Button("", qRect.x + qRect.w + margin, qRect.y, 200, 50);
	private final int infoHeight = 75;
	private final Pos fileButtonSize = new Pos(100, (infoHeight-margin) / 2);
	public final Button imageRect = new Button("", qRect.x + qRect.w + buttonRect.w + margin*2, qRect.y, 0, 0);
	private final int logoHeight = qRect.y - 3 * margin - infoHeight;
	private final Button scrollBar = new Button("",qRect.x + qRect.w - 2*margin, qRect.y + margin/2, 3*margin/2, 5*margin);
	private boolean scrolling = false;
	private ArrayList<RobotCommand> queue = new ArrayList<RobotCommand>();
	private ArrayList<RobotCommand> queueBuffer = null;
	private ArrayList<EventButton> eventButtons = new ArrayList<EventButton>();
	private ArrayList<EventButton> commandButtons = new ArrayList<EventButton>();//for resizing and repositioning
	private RobotCommand mouseCommand = null;
	private BufferedImage fieldImage = null;
	private String fieldImageName = "";
	private double fieldScaleFactor;
	private BufferedImage logoImage = null;
	private PointSelect currentPointSelect = null;
	public String scriptName = "";
	private int scrollPosition = 0;

	private List<HistoryEntry> history = new ArrayList<HistoryEntry>();
	private int historyPosition = -1;

	private EventButton removeButton;

	public MainWindow() {
		super(925, 675, "Cougar Script Editor");
		if (instance != null) {
			return;
		}
		instance = this;
		canvas.addMouseListener(this);
		canvas.addMouseMotionListener(this);
		canvas.addMouseWheelListener(this);
		canvas.addKeyListener(this);

		scrollBar.setColor(new Color(200, 200, 200));
		//event buttons
		Set<String> keySet = Commands.commandMap.keySet();
		String[] keys = keySet.toArray(new String[keySet.size()]);
		for (int i = 0; i < keys.length; i++) {
			String key = keys[i];
			Supplier<RobotCommand> supplier = Commands.commandMap.get(key);
			EventButton eb = new EventButton(key, buttonRect.x, qRect.y + i * (buttonRect.h + margin), buttonRect.w, buttonRect.h, true) {
				public void onClick() {
					mouseCommand = supplier.get();
				}
			};
			RobotCommand dummy = supplier.get();
			eb.setColor(dummy.getColor());
			eventButtons.add(eb);
			commandButtons.add(eb);
		}
		eventButtons.add(new EventButton("Undo (z)", qRect.x + qRect.w - 2*fileButtonSize.x - margin, qRect.y - margin - fileButtonSize.y, fileButtonSize.x, fileButtonSize.y, true)
			{@Override public void onClick() {undoChange();}});
		eventButtons.add(new EventButton("Redo (x)", qRect.x + qRect.w - fileButtonSize.x, qRect.y - margin - fileButtonSize.y, fileButtonSize.x, fileButtonSize.y, true)
			{@Override public void onClick() {redoChange();}});
		eventButtons.add(new EventButton("Export (s)", qRect.x + qRect.w - 2*fileButtonSize.x - margin, qRect.y - margin - infoHeight, fileButtonSize.x, fileButtonSize.y, true)
			{@Override public void onClick() {picker.exportToJSON(getScriptJSON());}});
		eventButtons.add(new EventButton("Import (o)", qRect.x + qRect.w - fileButtonSize.x, qRect.y - margin - infoHeight, fileButtonSize.x, fileButtonSize.y, true)
			{@Override public void onClick() {picker.importFromJSON(MainWindow.this);}});
		eventButtons.add(new EventButton("Paste (v)", buttonRect.x, buttonRect.y - infoHeight - margin, (buttonRect.w - margin) / 2, infoHeight, true)
			{@Override public void onClick() {pasteFromClipboard();}});
		eventButtons.add(new EventButton("Select Image", imageRect.x, imageRect.y - margin - fileButtonSize.y, fileButtonSize.x, fileButtonSize.y, true)
			{@Override public void onClick() {picker.selectFieldImage(MainWindow.this);}});
		removeButton = new EventButton("Remove (esc)", buttonRect.x + (buttonRect.w + margin) / 2, buttonRect.y - infoHeight - margin, (buttonRect.w - margin) / 2, infoHeight, true)
			{@Override public void onClick() {mouseCommand = null;}};
		eventButtons.add(removeButton);
		//load images
		try {
			logoImage = ImageIO.read(new File("scripteditor/src/resources/logo.png"));
			frame.setIconImage(new ImageIcon("scripteditor/src/resources/Icon.png").getImage());
			setFieldImage(ImageIO.read(new File("scripteditor/src/resources/47.350364343 FieldImage.png")), "47.350364343 FieldImage");
		} catch (IOException e) {
			e.printStackTrace();
		}
		addHistoryEntry('.');
		//add a set start position
		queue.add(Commands.commandMap.get("SetStartPosition").get());
		timer.start();
	}

	private void pasteFromClipboard() {
		try {
			mouseCommand = RobotCommand.getFromJSON(new JSONObject((String) Toolkit.getDefaultToolkit()
			.getSystemClipboard().getData(DataFlavor.stringFlavor)));
		} catch (JSONException e) {
			System.out.println("Invalid JSON");//TODO popup
		} catch (Exception e) {
			e.printStackTrace();
		}	
	}

	public void addHistoryEntry(char c) {
		for (int i = history.size() - 1; i > historyPosition; i--) {
			history.remove(i);
		}
		history.add(new HistoryEntry(c, getScriptJSON()));
		historyPosition++;
	}

	public void undoChange() {
		if (historyPosition > 0) {
			historyPosition--;
			setQueueFromJSON(history.get(historyPosition).getData());
		}
	}

	public void redoChange() {
		if (historyPosition < history.size() - 1) {
			historyPosition++;
			setQueueFromJSON(history.get(historyPosition).getData());
		}
	}

	public void clearHistory() {
		history.clear();
		historyPosition = -1;
	}

	private int getInsertBoxY() {
		int output = qRect.y + scrollPosition;
		for (int i = 0; i < queue.size(); i++) {
			RobotCommand command = queue.get(i);
			if (output + command.h + margin > m.y) {
				return output;
			}
			output += margin;
			output += command.h;
		}
		return output;
	}
	
	private void updateRobotPath() {
		Robot initial = Robot.getInitialRobot();
		if (queue.size() >= 1) {
			queue.get(0).updateRobot(initial);
		}
		for (int i = 1; i < queue.size(); i++) {
			queue.get(i).updateRobot(queue.get(i-1).getOutput());
		}
	}

	private void deselectPointSelect(boolean selectNext) {
		if (currentPointSelect != null) {
			highlightPointSelects(PointSelect.DEFAULT_COLOR);
			if (selectNext && currentPointSelect.getNext() != null) {//automatically select the next one
				enterPointSelection(currentPointSelect.getNext());
			} else {
				currentPointSelect = null;
			}
		}
	}
	
	public void enterPointSelection(PointSelect ps) {
		if (mouseCommand == null) {
			currentPointSelect = ps;
			highlightPointSelects(PointSelect.HIGHLIGHT_COLOR);
			ps.setColor(new Color(255, 184, 184));
		}
	}

	private void highlightPointSelects(Color c) {
		for (PointSelect ps : getPointSelects()) {
			ps.setColor(c);
		}
	}

	private void scrollQueue(int amount) {
		scrollPosition += amount;
		int max = -1 * (10 * (qRect.h - scrollBar.h - margin));
		if (scrollPosition > 0) {
			scrollPosition = 0;
		} else if (scrollPosition < max) {
			scrollPosition = max;
		}
	}

	public void update() {
		//resize
		qRect.h = h - qRect.y - 2 * margin - frame.getInsets().top;
		double buttonListh = frame.getSize().height - insets.top - insets.bottom - 3*margin - logoHeight - infoHeight;
		double tempHeight = buttonListh / commandButtons.size();
		for (int i = 0; i < commandButtons.size(); i++) {
			EventButton eb = commandButtons.get(i);
			eb.h = (int)(tempHeight - margin);
			eb.y = (int)(buttonRect.y + i * tempHeight);
		}
		//background
		g.setColor(backgroundColor);
		g.fillRect(0, 0, w, h);
		//draw field image
		if (fieldImage != null) {
			g.setColor(Color.black);
			g.drawString("Current image: " + fieldImageName, imageRect.x, imageRect.y - margin - fileButtonSize.y - 2);
			imageRect.w = frame.getSize().width - insets.left - insets.right - 4*margin - qRect.w - buttonRect.w;
			imageRect.h = (int)((double)imageRect.w * fieldImage.getHeight() / fieldImage.getWidth());
			int maxh = frame.getSize().height - insets.top - insets.bottom - 4*margin - logoHeight - infoHeight;
			if (imageRect.h > maxh) {
				imageRect.h = maxh;
				imageRect.w = (int)((double)imageRect.h * fieldImage.getWidth() / fieldImage.getHeight());
			}
			g.drawImage(fieldImage, imageRect.x, imageRect.y, imageRect.w, imageRect.h, null);
		}
		if (imageRect.inBounds(m)) {
			g.setColor(Color.darkGray);
			g.drawLine(m.x, imageRect.y, m.x, imageRect.y + imageRect.h);
			g.drawLine(imageRect.x, m.y, imageRect.x + imageRect.w, m.y);
			Point mfeet = new Point(m).toFeet().getRounded();
			Pos ft = new Pos((int)mfeet.x, (int)mfeet.y);
			Point inches = new Point(12*(mfeet.x - ft.x), 12*(mfeet.y - ft.y)).getRounded();
			g.drawString("x: "+mfeet.x, imageRect.x + imageRect.w - 110, imageRect.y - g.getFontMetrics().getHeight() - margin - 2);
			g.drawString(((inches.x < 0) ? "-":" ")+Math.abs(ft.x)+"\' ", imageRect.x + imageRect.w - 60, imageRect.y - g.getFontMetrics().getHeight() - margin - 2);
			g.drawString(+Math.abs(inches.x)+"\'\'", imageRect.x + imageRect.w - 35, imageRect.y - g.getFontMetrics().getHeight() - margin - 2);
			g.drawString("y: "+mfeet.y, imageRect.x + imageRect.w - 110, imageRect.y - margin - 2);
			g.drawString(((inches.y < 0) ? "-":" ")+Math.abs(ft.y)+"\' ", imageRect.x + imageRect.w - 60, imageRect.y - margin - 2);
			g.drawString(+Math.abs(inches.y)+"\'\'", imageRect.x + imageRect.w - 35, imageRect.y - margin - 2);
		}
		//update robot path
		updateRobotPath();
		for (int i = 0; i < queue.size(); i++) {
			queue.get(i).drawPreview(g);
		}
		//draw queue
		qRect.draw(g);
		int currentY = qRect.y;
		for (int i = 0; i < queue.size(); i++) {
			currentY += margin;
			RobotCommand command = queue.get(i);
			command.update(new Pos(qRect.x + 3 * margin, currentY + scrollPosition));
			g.setColor(Color.black);
			g.drawString(Integer.toString(i), qRect.x + margin, currentY + margin + scrollPosition);
			currentY += command.h;
			command.draw(g);
		}
		g.setColor(Color.black);
		if (mouseCommand != null && m.x > qRect.x && m.x < qRect.x+qRect.w && m.y > qRect.y && m.y < qRect.y+qRect.h) {
			//insert line
			g.setColor(insertColor);
			g.fillRect(qRect.x+1, getInsertBoxY()+1, qRect.w-1, margin-1);
		}
		//Mask the part of the queue that goes above and below
		g.setColor(backgroundColor);
		g.fillRect(0, 0, qRect.x + qRect.w, qRect.y);
		g.fillRect(0, qRect.y + qRect.h + 1, qRect.x + qRect.w, 2*margin);
		//display top info
		g.setColor(Color.black);
		g.drawString("Script Name: " + scriptName, qRect.x, qRect.y - margin - 2*g.getFontMetrics().getHeight());
		g.drawString("Script length: " + Integer.toString(queue.size()), qRect.x, qRect.y - margin - g.getFontMetrics().getHeight());
		String historyString = "History: ";
		int stringLength = 20;
		int lower = Math.max(0, Math.min(historyPosition - stringLength/2, history.size()-stringLength));
		int upper = Math.min(lower + stringLength, history.size());
		if (lower > 0) {
			lower += 2;
			historyString += "["+lower+"] ";
			lower += Integer.toString(lower).length();
		}
		boolean addEnding = false;
		if (upper < history.size()) {
			addEnding = true;
			upper -= Integer.toString(upper).length();
		}
		for (int i = lower; i < upper; i++) {
			historyString += history.get(i).getCharacter();
			if (i == historyPosition) {
				historyString += " | ";
			}
		}
		if (addEnding) {
			historyString += " ["+(history.size()-upper)+"]";
		}
		g.drawString(historyString, qRect.x, qRect.y - margin);
		//draw scrollbar
		g.setColor(Color.black);
		scrollBar.y = qRect.y - (int)(0.1*scrollPosition) + margin/2;
		scrollBar.draw(g);
		//draw lambdaButtons
		for (EventButton button : eventButtons) {
			button.draw(g);
		}
		//draw remove button
		if (mouseCommand == null) {
			removeButton.setColor(Button.DEFAULT_BACKGROUND_COLOR);
		} else {
			removeButton.setColor(REMOVE_COLOR);
		}
		//draw logo
		if (logoImage != null) {
			g.drawImage(logoImage, margin, margin, (int)(logoImage.getWidth() * logoHeight / logoImage.getHeight()), logoHeight, null);
		}
		//draw mouse command
		if (mouseCommand != null) {
			g.setColor(Color.black);
			mouseCommand.update(m);
			mouseCommand.draw(g);
		}
		//point selection
		if (currentPointSelect != null) {
			g.setColor(Color.black);
			//g.drawString("Select a point on the field image", imageRect.x, imageRect.y + g.getFontMetrics().getHeight());
			int nextPoints = 0;
			for (PointSelect ps = currentPointSelect; ps != null; ps = ps.getNext()) {
				nextPoints++;
			}
			if (imageRect.inBounds(mEvent)) {
				g.setColor(Color.blue);
				String str = Integer.toString(nextPoints);
				g.drawString(str, m.x - 3 - g.getFontMetrics().stringWidth(str), m.y - 3);
			}
		}
		//update queue
		if (queueBuffer != null) {
			queue = queueBuffer;
			queueBuffer = null;
		}
	}
	//listener methods
	@Override
	public void mouseDragged(MouseEvent e) {
		mouseMoved(e);
		if (scrolling) {
			scrollQueue(-10 * (m.y - prevM.y));
		}
	}
	@Override
	public void mouseMoved(MouseEvent e) {
		mEvent = e;
		prevM.x = m.x;
		prevM.y = m.y;
		m.x = e.getX();
		m.y = e.getY();
		for (int i = 0; i < queue.size(); i++) {
			queue.get(i).processMouseMove(e);
		}
		for (int i = 0; i < eventButtons.size(); i++) {
			eventButtons.get(i).processMouseMove(e);
		}
	}
	@Override
	public void mouseClicked(MouseEvent e) {}
	@Override
	public void mouseEntered(MouseEvent arg0) {}
	@Override
	public void mouseExited(MouseEvent arg0) {}
	@Override
	public void mousePressed(MouseEvent e) {
		mouseMoved(e);//update hover
		if (scrollBar.inBounds(e)) {
			scrolling = true;
			return;
		}
		scrolling = false;
		//deselect text field
		TextField.deselectTextField();
		//run lambdaButtons
		for (int i = 0; i < eventButtons.size(); i++) {
			eventButtons.get(i).processClick(e);
		}
		//point selection
		if (currentPointSelect != null) {
			for (PointSelect ps : getPointSelects()) {
				if (ps.inBounds(e)) {
					currentPointSelect.setValue(ps.getValue());
					MainWindow.getInstance().addHistoryEntry('*');
					deselectPointSelect(true);
					return;
				}
			}
			if (imageRect.inBounds(e)) {
				currentPointSelect.setValue(new Point(m).toFeet());
				MainWindow.getInstance().addHistoryEntry('*');
				deselectPointSelect(true);
				return;
			}
			deselectPointSelect(false);
			return;
		}
		//queue
		RobotCommand toPickup = null;
		if (qRect.inBounds(e)) {
			for (int i = 0; i < queue.size(); i++) {
				RobotCommand command = queue.get(i);
				Element clicked = command.processClick(e);
				if (command == clicked) {
					toPickup = command;
					queue.remove(command);
					addHistoryEntry('q');
				}
				if (clicked != null && !(clicked instanceof RobotCommand)) {
					return;
				}
			}
			//put into queue
			if (mouseCommand != null) {
				int currentY = qRect.y + scrollPosition;
				int indexToInsert = queue.size();
				for (int i = 0; i < queue.size(); i++) {
					if (e.getY() < currentY + queue.get(i).h + margin) {
						indexToInsert = i;
						break;
					}
					currentY += queue.get(i).h + margin;
				}
				queue.add(indexToInsert, mouseCommand);
				addHistoryEntry('Q');
			}
			mouseCommand = toPickup;
			return;
		}
	}
	@Override
	public void mouseReleased(MouseEvent arg0) {}
	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
		scrollQueue(-e.getWheelRotation() * 25);
		if (mEvent != null) {
			mouseMoved(mEvent);
		}
	}
	@Override
	public void keyPressed(KeyEvent e) {
		int k = e.getKeyCode();
		if (k == KeyEvent.VK_ESCAPE) {//esc
			mouseCommand = null;
			deselectPointSelect(false);
			TextField.deselectTextField();
		}
		if (TextField.getSelectedTextField() == null) {//hotkeys
			if (k == KeyEvent.VK_V) {//paste
				pasteFromClipboard();
			} else if (k == KeyEvent.VK_Z) {//undo
				undoChange();
			} else if (k == KeyEvent.VK_X) {//redo
				redoChange();
			} else if (k == KeyEvent.VK_S) {//save
				picker.exportToJSON(getScriptJSON());
			} else if (k == KeyEvent.VK_O) {//open
				picker.importFromJSON(this);
			}
		} else {
			if (k == KeyEvent.VK_ENTER) {
				TextField.deselectTextField();
			} else {
				TextField.getSelectedTextField().processKeyEvent(e);
			}
		}
	}
	@Override
	public void keyReleased(KeyEvent arg0) {}
	@Override
	public void keyTyped(KeyEvent arg0) {}

	/* Getters and setters */
	
	public static MainWindow getInstance() {
		return instance;
	}

	public void setFieldImage(BufferedImage image, String name) {
		int i = name.indexOf(' ');
		try {
			if (i > -1) {
				fieldScaleFactor = Double.parseDouble(name.substring(0, i));
			}
		} catch (NumberFormatException e) {
			fieldScaleFactor = 10.0;
		}
		fieldImage = image;
		fieldImageName = name;
	}

	public BufferedImage getFieldImage() {
		return fieldImage;
	}

	public double getFieldImageScale() {
		double unscaledWidth = 1;
		if (fieldImage != null)  {
			unscaledWidth = fieldImage.getWidth();
		}
		return fieldScaleFactor * (imageRect.w / unscaledWidth);
	}

	public FontMetrics getFontMetrics() {
		return g.getFontMetrics();
	}

	private JSONArray getScriptJSON() {
		JSONArray commandList = new JSONArray();
		for (int i = 0; i < queue.size(); i++) {
			RobotCommand command = queue.get(i);
			commandList.put(command.getJSON());
		}
		return commandList;
	}

	public void setQueueFromJSON(JSONArray commandList) {
		queueBuffer = new ArrayList<RobotCommand>();
		for (int i = 0; i < commandList.length(); i++) {
			JSONObject command = commandList.getJSONObject(i);
			queueBuffer.add(RobotCommand.getFromJSON(command));
		}
	}

	private List<PointSelect> getPointSelects() {
		List<Element> cList = new ArrayList<Element>();
		for (int i = 0; i < queue.size(); i++) {
			cList.add(queue.get(i));
		}
		List<PointSelect> output = new ArrayList<PointSelect>();
		return getPointSelects(cList, output);
	}

	private List<PointSelect> getPointSelects(List<Element> children, List<PointSelect> output) {
		for (Element element : children) {
			if (element instanceof PointSelect) {
				output.add((PointSelect) element);
			}
			getPointSelects(element.getChildren(), output);
		}
		return output;
	}

	public RobotCommand getMouseCommand() {
		return mouseCommand;
	}

	public void setMouseCommand(RobotCommand c) {
		mouseCommand = c;
	}

	private class HistoryEntry {
		/**
		 * . = open file / start
		 * Q = put into queue
		 * q = take from queue
		 * p = parallel command interaction
		 * n = number field
		 * b = toggle button
		 * * = point selection
		 * + = add to list
		 * - = remove from list
		 **/
		private char c;
		private JSONArray data;
		public HistoryEntry(char c, JSONArray data) {
			this.c = c;
			this.data = data;
		}
		public char getCharacter() {
			return c;
		}
		public JSONArray getData() {
			return data;
		}
	}
}