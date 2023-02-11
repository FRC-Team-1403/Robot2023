package gui;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferStrategy;

import javax.swing.JFrame;
import javax.swing.Timer;

/**
 * A supporting class to handle running the window.
 * 
 * @author Brandon C.
 */
public abstract class Window{
	protected int w;
	protected int h;
	protected String name;
	protected JFrame frame;
	protected Canvas canvas;
	protected BufferStrategy bs;
	protected Graphics g;
	protected Insets insets;
	protected Timer timer;
	
	public Window (int width, int height, String name) {
		this.w = width;
		this.h = height;
		this.name = name;
		frame = new JFrame(name);
		frame.setSize(new Dimension(w, h));
		frame.setMinimumSize(new Dimension(w, h));
		frame.setVisible(true);
		frame.setResizable(true);
		canvas = new Canvas();
		canvas.setSize(new Dimension(w, h));
		canvas.setBackground(new Color(255, 255, 255));
		frame.add(canvas);
		canvas.createBufferStrategy(3);
		bs = canvas.getBufferStrategy();
		insets = frame.getInsets();
		timer = new Timer(16, new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				w = frame.getWidth();
				h = frame.getHeight();
				g = bs.getDrawGraphics();
				g.clearRect(0, 0, w, h);
				update();
				bs.show();
				g.dispose();
			}
		});
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				timer.stop();
			}

			@Override
			public void windowClosed(WindowEvent e) {
				timer.stop();
			}

		});
	}
	
	/**
	 * Called once per loop.
	 */
	public abstract void update();
}