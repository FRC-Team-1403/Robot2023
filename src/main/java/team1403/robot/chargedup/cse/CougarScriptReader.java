package team1403.robot.chargedup.cse;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

/**
 * A utility class that helps with reading Cougar Scripts from JSON format.
 * Commands must be registered by calling the registerCommand method.
 * Scripts can be loaded using the importScript method.
 * Scripts mmust be placed directly in the deploy directory to be imported.
 *
 * @author Brandon C.
 */
public class CougarScriptReader {

  private static CougarScriptReader _instance;

  private Map<String, Function<CougarScriptObject, Command>> m_commandMap;
  Consumer<Pose2d> onStartPose;

  /**
   * Constructor for CougarScriptReader.
   *
   * @param toRun sets the start position of your robot. Called when a
   *              SequentialCommandGroup (created by the CougarScriptReader) is
   *              scheduled. The start position is read from the setStartPosition
   *              Command in a Cougar Script.
   *              When initilizing your CougarScriptReader, make sure to define
   *              this so that your robot can set its starting position for the
   *              drivepath
   *              reader = new CougarScriptReader((Pose2d startPose) -> {
   *              methodToSetDriveTrainStartPose(startPose);
   *              });
   */
  public CougarScriptReader(Consumer<Pose2d> toRun) {
    if (_instance != null) {
      return;
    }
    _instance = this;
    onStartPose = toRun;

    // register parallelcommand automatically
    m_commandMap = new HashMap<String, Function<CougarScriptObject, Command>>();

    registerCommand("ParallelCommand", (CougarScriptObject p) -> {
      ArrayList<Command> commandsToRun = new ArrayList<>();
      ArrayList<Command> endCommands = new ArrayList<Command>();
      JSONArray commandListJson = p.getJsonArray("Commands");
      for (int i = 0; i < commandListJson.length(); i++) {
        JSONObject parallelFieldJson = commandListJson.getJSONObject(i);
        boolean endCondition = parallelFieldJson.getBoolean("EndCondition");
        Command currentCommand = parseCommandFromJson(parallelFieldJson.getJSONObject("Command"));
        if (currentCommand != null) {
          commandsToRun.add(currentCommand);
          if (endCondition) {
            endCommands.add(currentCommand);
          }
        }
      }
      return new ParallelCommand(commandsToRun, endCommands);
    });
    registerCommand("SequentialCommand", (CougarScriptObject p) -> {
      ArrayList<Command> commandsToRun = new ArrayList<>();
      JSONArray commandListJson = p.getJsonArray("Commands");
      for (int i = 0; i < commandListJson.length(); i++) {
        JSONObject seqFieldJson = commandListJson.getJSONObject(i);
        Command currentCommand = parseCommandFromJson(seqFieldJson.getJSONObject("Command"));
        if (currentCommand != null) {
          commandsToRun.add(currentCommand);
        }
      }
      return new SequentialCommandGroup(commandsToRun);
    });
  }

  /**
   * Creates a sequential command group from the cougar script JSON file.
   *
   * @param autoName The filename of the script, located in the deploy directory
   *                 NOTE: do not create directories in the deploy directory.
   * @return Sequential command group containing all the commands from the script.
   */
  public SequentialCommandGroup importScript(String autoName) {
    String filepath = Filesystem.getDeployDirectory() + "/" + autoName;
    ArrayList<Command> commands = new ArrayList<Command>();
    Pose2d startPose = new Pose2d();
    File fileToOpen = new File(filepath);
    try (Scanner sc = new Scanner(fileToOpen);) {
      String data = "";
      while (sc.hasNextLine()) {
        data += sc.nextLine();
      }
      sc.close();
      JSONTokener tokener = new JSONTokener(data);
      JSONArray commandList = new JSONArray(tokener);
      for (int i = 0; i < commandList.length(); i++) {
        JSONObject commandJson = commandList.getJSONObject(i);
        if (((String) commandJson.get("CommandName")).equals("SetStartPosition")) {
          JSONObject parameters = (JSONObject) commandJson.get("Parameters");
          startPose = new Pose2d(
              parameters.getDouble("x"),
              parameters.getDouble("y"),
              new Rotation2d(parameters.getDouble("angle")));
        }
        Command command = parseCommandFromJson(commandJson);
        if (command != null) {
          commands.add(command);
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    return new SequentialCommandGroup(startPose, onStartPose, 
        commands.toArray(new Command[commands.size()]));
  }

  private Command parseCommandFromJson(JSONObject commandJson) {
    String commandName = (String) commandJson.get("CommandName");
    if (commandName.equals("SetStartPosition")) {
      return null;
    }
    JSONObject parameters = (JSONObject) commandJson.get("Parameters");
    Function<CougarScriptObject, Command> f = m_commandMap.get(commandName);
    if (f != null) {
      return f.apply(new CougarScriptObject(parameters));
    } else {
      return null;
    }
  }

  /**
   * Registers a command to be able to create it from the Cougar Script.
   *
   * @param commandName The name of the command, matching on the Script Editor
   *                    exactly (case sensitive)
   * @param f           The function that determines how to create the command
   *                    based on the info in the Cougar Script.
   *                    The function takes a CougarScriptObject and returns a
   *                    Command. The Cougar script object enables
   *                    access of the different parameters stored in JSON for that
   *                    command, and the Function will return
   *                    a command. The parameters for the command can be accessed
   *                    through the CougarScriptObject and
   *                    these values can be used to specify how to create the
   *                    command in the function. These parameters
   *                    should match with the ones specified in the Script Editor
   *                    (case sensitive).
   *
   *                    <p>Example:
   *
   *                    <p>In the script editor, there is a robotCommand called
   *                    "myDrive" which has a double parameter
   *                    "Distance" and a boolean parameter "Direction" (forwards
   *                    or backwards).
   *
   *                    <p>In the robot code, there is a command called
   *                    "ExampleDriveCommand" with the following constructor:
   *                    public ExampleDriveCommand(Subsystem driveSubsystem,
   *                    double distance, boolean direction)
   *
   *                    <p>Here's an example of how this command could be registered:
   *
   *                    <p>registerCommand("myDrive", (CougarScriptObject parameters)
   *                    -> {
   *
   *                    <p>Subsystem driveSubsystem =
   *                    robotContainer.getDriveSubsystem();
   
   *                    <p>double distance = parameters.getDouble("Distance");
   *
   *                    <p>boolean direction = parameters.getBoolean("Direction");
   *
   *                    <p>Command output = new ExampleDriveCommand(driveSubsystem,
   *                    distance, direction);
   *
   *                    <p>return output;
   *
   *                    <p>});
   * 
   *                    Note that the lambda expression takes a CougarScriptObject
   *                    and returns a Command.
   * 
   */
  public void registerCommand(String commandName, Function<CougarScriptObject, Command> f) {
    m_commandMap.put(commandName, f);
  }
}