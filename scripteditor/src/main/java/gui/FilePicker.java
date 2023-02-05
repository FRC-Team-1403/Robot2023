package gui;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import javax.imageio.ImageIO;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.filechooser.FileNameExtensionFilter;

import org.json.JSONArray;
import org.json.JSONTokener;

/**
 * Contains the JFileChooser used to access stored Cougar scripts in JSON format as well as select an image for the preview.
 * 
 * @author Brandon C.
 */
public class FilePicker {
	
	private final JFrame chooserFrame;
	private final JFileChooser fileChooser;
	private final FileNameExtensionFilter json = new FileNameExtensionFilter("Json scripts", "json");
	private final FileNameExtensionFilter image = new FileNameExtensionFilter("Images", "jpg", "png");
	
	/**
	 * Initializes the FilePicker.
	 */
	public FilePicker() {
		chooserFrame = new JFrame();
		fileChooser = new JFileChooser();
	}
	
	/**
	 * Creates or modifies a file to store the script information.
	 * The file is selected using a JFileChooser.
	 * @param commandList The JSONArray containing the script information.
	 */
	public void exportToJSON(JSONArray commandList) {
		fileChooser.setFileFilter(json);
		fileChooser.setDialogTitle("Save to a file");
		int userSelection = fileChooser.showSaveDialog(chooserFrame);
		if (userSelection == JFileChooser.APPROVE_OPTION) {
			File saveFile = fileChooser.getSelectedFile();
			MainWindow.getInstance().scriptName = saveFile.getName();
			String path = saveFile.getAbsolutePath();
			if (path.length() > 5 && !path.substring(path.length() - 5, path.length()).equals(".json")) {
				path += ".json";
			}
			try (FileWriter fileWriter = new FileWriter(path)) {
				commandList.write(fileWriter, 4, 0);
				fileWriter.flush();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Loads a JSON file containing a script and sets the queue based on the information in the script file.
	 * @param mw The MainWindow to modify.
	 */
	public void importFromJSON(MainWindow mw) {
		fileChooser.setFileFilter(json);
		fileChooser.setDialogTitle("Specify a file to open");
		int userSelection = fileChooser.showOpenDialog(chooserFrame);
		if (userSelection == JFileChooser.APPROVE_OPTION) {
			File fileToOpen = fileChooser.getSelectedFile();
			mw.scriptName = fileToOpen.getName();
			try (Scanner sc = new Scanner(fileToOpen);) {
				String data = "";
				while (sc.hasNextLine()) {
					data += sc.nextLine();
				}
				sc.close();
				JSONTokener tokener = new JSONTokener(data);
				JSONArray commandList = new JSONArray(tokener);
				mw.setQueueFromJSON(commandList);
				mw.addHistoryEntry('O');
				mw.clearHistory();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Sets the field image to use for the preview.
	 * The image must have the pixels per foot ratio as a valid double in the beginning of the title, delimited with a space.
	 * @param mw The MainWindow to modify.
	 */
	public void selectFieldImage(MainWindow mw) {
		fileChooser.setFileFilter(image);
		fileChooser.setDialogTitle("Select a field image to display");
		int userSelection = fileChooser.showOpenDialog(chooserFrame);
		if (userSelection == JFileChooser.APPROVE_OPTION) {
			try {
				File selected = fileChooser.getSelectedFile();
				String imageName = selected.getName();
				mw.setFieldImage(ImageIO.read(selected), imageName);
			} catch (IOException e) {
				e.printStackTrace();
			} 
		}
	}
}
