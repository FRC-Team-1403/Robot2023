package commands;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.function.Supplier;

import preview.Robot;
import preview.Turret;
import util.Point;

/**
 * A class that contains the commandMap, which stores suppliers for the different RobotCommands.
 * 
 * @author Brandon C.
 */
public class Commands {
    
	public static final Color DRIVE_COLOR = new Color(192, 139, 252);//light purple
	public static final Color TURRET_COLOR = new Color(139, 212, 252);//light blue
	public static final Color SHOOT_COLOR = new Color(163, 252, 139);//light green
    public static final Color MISC_COLOR = new Color(252, 184, 139);//light red-orange
    
    /**
	 * A map containing presets for the RobotCommands on the 2021 1403 robot.
	 * The two MISC ones (SetStartPosition and ParallelCommand) are required.
	 */
	public static final SortedMap<String, Supplier<RobotCommand>> commandMap = new TreeMap<String,Supplier<RobotCommand>>();
	static {
		commandMap.put("SetStartPosition", () -> new RobotCommand("SetStartPosition", MISC_COLOR, 
			new NumberField("angle"), new PointSelect("position")
		){
			@Override
			public Robot applyCommand(Robot initial) {
				Robot output = initial.getCopy();
				Point p = ((PointSelect) getInteractable("position")).getValue();
				output.setPosition(new Point(p.x, p.y));
				output.setAngle(getNumberFromName("angle").getValue());
				return output;
			}
			@Override
			public void drawPreview(Graphics g) {
				g.setColor(Color.red);
				getOutput().getPosition().toPixels().draw(g);
				if (getHovered()) {
					getOutput().draw(g);
				}
			}
		});
		commandMap.put("ParallelCommand", ()->new RobotCommand("ParallelCommand", Commands.MISC_COLOR, 
			new InteractableList<ParallelCommandField, RobotCommand>("Commands", "Command", true, ParallelCommandField::new)
		){
			@Override
			@SuppressWarnings("unchecked")
			public Robot applyCommand(Robot initial) {
				Robot output = initial;
				List<ParallelCommandField> list = ((InteractableList<ParallelCommandField, RobotCommand>)getInteractable("Commands")).getValue();
				for (int i = 0; i < list.size(); i++) {
					ParallelCommandField field = list.get(i);
					if (field.hasCommand()) {
						output = field.getValue().applyCommand(output);
					}
				}
				return output;
			}
		});
		commandMap.put("SetTurretAngle", () -> new RobotCommand("SetTurretAngle", TURRET_COLOR, new NumberField("Angle")) {
			@Override
			public Robot applyCommand(Robot initial) {
				Robot output = initial.getCopy();
				((Turret) output.getSubsystem("Turret")).setAngle(getNumberFromName("Angle").getValue());
				return output;
			}
		});
		commandMap.put("DrivePath", ()->new RobotCommand("DrivePath", DRIVE_COLOR, 
            new NumberField("StartAngle"),
			new NumberField("EndAngle"), 
			new ToggleButton("Inverted"),
            new InteractableList<PointSelect, Point>("Waypoints", "Waypoint", false, PointSelect::new)
        ) {
			List<Point> points;
			@Override
			@SuppressWarnings("unchecked")
			public Robot applyCommand(Robot initial) {
				Robot output = initial.getCopy();
				List<PointSelect> pointSelections = ((InteractableList<PointSelect, Point>) getInteractable("Waypoints")).getValue();
				points = new ArrayList<Point>();
				pointSelections.forEach((PointSelect ps)->points.add(ps.getValue()));
				if (points.size() > 0) {
					initial.setPosition(points.get(0));
					output.setPosition(points.get(points.size()-1));
				}
				initial.setAngle(getNumberFromName("StartAngle").getValue());
				output.setAngle(getNumberFromName("EndAngle").getValue());
				
				return output;
			}
			@Override
			public void drawPreview(Graphics g) {
				super.drawPreview(g);
				List<Point> scaledPoints = new ArrayList<Point>();
				for (int i = 0; i < points.size(); i++) {
					scaledPoints.add(points.get(i).toPixels());
				}
				g.setColor(Color.red);
				for (int i = 0; i < points.size()-1; i++) {
					scaledPoints.get(i).drawLine(g, scaledPoints.get(i+1));
				}
				if (getHovered()) {
					g.setColor(Color.orange);
					for (int i = 0; i < scaledPoints.size(); i++) {
						Point p = scaledPoints.get(i);
						g.drawString(Integer.toString(i), (int)p.x-3, (int) p.y+3);
					}
				}
			}
		});

		commandMap.put("SwerveDrivePath", ()->new RobotCommand("SwerveDrivePath", DRIVE_COLOR, 
            new NumberField("StartAngle"),
			new NumberField("EndAngle"),
            new InteractableList<PointSelect, Point>("Waypoints", "Waypoint", false, PointSelect::new)
        ) {
			List<Point> points;
			@Override
			@SuppressWarnings("unchecked")
			public Robot applyCommand(Robot initial) {
				Robot output = initial.getCopy();
				List<PointSelect> pointSelections = ((InteractableList<PointSelect, Point>) getInteractable("Waypoints")).getValue();
				points = new ArrayList<Point>();
				pointSelections.forEach((PointSelect ps)->points.add(ps.getValue()));
				if (points.size() > 0) {
					initial.setPosition(points.get(0));
					output.setPosition(points.get(points.size()-1));
				}
				initial.setAngle(getNumberFromName("StartAngle").getValue());
				output.setAngle(getNumberFromName("EndAngle").getValue());
				
				return output;
			}
			@Override
			public void drawPreview(Graphics g) {
				super.drawPreview(g);
				List<Point> scaledPoints = new ArrayList<Point>();
				for (int i = 0; i < points.size(); i++) {
					scaledPoints.add(points.get(i).toPixels());
				}
				g.setColor(Color.red);
				for (int i = 0; i < points.size()-1; i++) {
					scaledPoints.get(i).drawLine(g, scaledPoints.get(i+1));
				}
				if (getHovered()) {
					g.setColor(Color.orange);
					for (int i = 0; i < scaledPoints.size(); i++) {
						Point p = scaledPoints.get(i);
						g.drawString(Integer.toString(i), (int)p.x-3, (int) p.y+3);
					}
				}
			}
		});
	}
}