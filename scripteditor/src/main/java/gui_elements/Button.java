package gui_elements;

import java.awt.Color;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;

import util.Pos;

/**
 * The most basic form of UI element with fucntionality for displaying itself as a rectangle with a String of text.
 * Contains a list of child Elements which, once added, will automatically be drawn and process mouse events.
 * 
 * @author Brandon C.
 */
public class Button implements Element{

	public static final Color DEFAULT_BACKGROUND_COLOR = Color.white;

	public int x, y, w, h;

	private Color backgroundColor;
	private Color hoverColor;
	private boolean hovered;
	private String text;
	private List<Element> children;
	private boolean centerText = false;
	private Pos relativePos;
	private Pos defaultSize;
	
	/**
	 * Create a new Button.
	 * @param text The text to display on the button.
	 * @param x The x position of the Button.
	 * @param y The y position of the Button.
	 * @param w The width of the Button.
	 * @param h The height of the Button.
	 */
	public Button(String text, int x, int y, int w, int h) {
		this.text = text;
		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
		this.relativePos = new Pos(x, y);
		this.defaultSize = new Pos(w, h);
		children = new ArrayList<Element>();
		hovered = false;
		setColor(DEFAULT_BACKGROUND_COLOR);
	}

	/**
	 * Sets whether the text of the button is centered or not
	 * @param center Whether the text should be centered or not
	 */
	public void setCentered(boolean center) {
		this.centerText = center;
	}
	
	/**
	 * Sets the background color and color when hovered (90% the brightness).
	 * @param c The background color.
	 */
	public void setColor(Color c) {
		backgroundColor = c;
		float[] hsb = Color.RGBtoHSB(c.getRed(), c.getGreen(), c.getBlue(), null);
		int rgb = Color.HSBtoRGB(hsb[0], hsb[1], (float)(hsb[2] * 0.9));
		hoverColor = new Color(rgb);
	}

	/**
	 * Gets the backgorund color of the button.
	 * @return The background color.
	 */
	public Color getColor() {
		return backgroundColor;
	}

	/**
	 * {@inheritDoc}
	 * Uses the hovor color if hovered.
	 */
	@Override
	public void draw(Graphics g) {
		if (hovered) {
			draw(g, hoverColor);
		} else {
			draw(g, backgroundColor);
		}
	}
	
	/**
	 * Draws the button with a given color. Any children will also be drawn.
	 * @param g The graphics object to draw to.
	 * @param backgroundColor The color to draw the background with.
	 */
	public void draw(Graphics g, Color backgroundColor) {
		g.setColor(backgroundColor);
		g.fillRect(x, y, w, h);
		g.setColor(hoverColor);
		g.drawRect(x, y, w, h);
		g.setColor(Color.black);
		if (centerText) {
			FontMetrics fm = g.getFontMetrics();
			g.drawString(text, x + w/2 - fm.stringWidth(text)/2, y + h/2 + fm.getHeight()/2-3);
		} else {
			g.drawString(text, x + 3, y + g.getFontMetrics().getHeight()-3);
		}
		for (int i = 0; i < children.size(); i++) {
			children.get(i).draw(g);
		}
	}

	/**
	 * Sets the text of the button.
	 * @param text The String to use.
	 */
	public void setText(String text) {
		this.text = text;
	}

	/**
	 * Gets the text of the button.
	 * @return The text as a String.
	 */
	public String getText() {
		return text;
	}

	/**
	 * Gets whether or not the button is hovered.
	 * @return hovered.
	 */
	public boolean getHovered() {
		return hovered;
	}
	
	@Override
    public boolean inBounds (MouseEvent e) {
		return (e.getX() >= x && e.getX() <= x + w) && (e.getY() >= y && e.getY() <= y + h);
	}
	
	/**
	 * Gets whether the Pos is within the bounds of the button.
	 * @param p The position to use.
	 * @return Whether the Pos is within the bounds of the button or not.
	 */
	public boolean inBounds(Pos p) {
		return (p.x >= x && p.x <= x + w) && (p.y >= y && p.y <= y + h);
	}

	@Override
	public void processMouseMove(MouseEvent e) {
		hovered = inBounds(e);
		Element.super.processMouseMove(e);
	}

	@Override
    public Pos getSize() {
		return new Pos(w, h);
	}

	@Override
	public Pos getDefaultSize() {
		return defaultSize;
	}

	@Override
	public void setSize(Pos p) {
		w = p.x;
		h = p.y;
	}

	@Override
	public List<Element> getChildren() {
		return children;
	}

	@Override
	public void setRelativePosition(Pos p) {
		this.relativePos = p;
	}

	@Override
	public Pos getRelativePosition() {
		return this.relativePos;
	}

	@Override
	public Pos getPosition() {
		return new Pos(x, y);
	}

	@Override
	public void setPosition(Pos p) {
		this.x = p.x;
		this.y = p.y;
	}
}
