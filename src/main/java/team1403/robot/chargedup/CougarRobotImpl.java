package team1403.robot.chargedup;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarRobot;
import team1403.lib.util.CougarLogger;
import team1403.robot.chargedup.cse.CougarScriptObject;
import team1403.robot.chargedup.cse.CougarScriptReader;
import team1403.robot.chargedup.photonvision.PhotonVisionSubsystem;
import team1403.robot.chargedup.swerve.SwerveAutoBalanceYaw;
import team1403.robot.chargedup.swerve.SwerveCommand;
import team1403.robot.chargedup.swerve.SwerveDrivePath;
import team1403.robot.chargedup.swerve.SwerveSubsystem;
import team1403.robot.chargedup.RobotConfig.Operator;
import team1403.robot.chargedup.StateManager.GamePiece;
import team1403.robot.chargedup.StateManager.LED;
import team1403.robot.chargedup.arm.ArmState;
import team1403.robot.chargedup.arm.ArmStateGroup;
import team1403.robot.chargedup.arm.ArmSubsystem;
import team1403.robot.chargedup.arm.ManualArmCommand;
import team1403.robot.chargedup.arm.RunIntake;
import team1403.robot.chargedup.arm.SequentialMoveArmCommand;
import team1403.robot.chargedup.arm.SetpointArmCommand;
import team1403.robot.chargedup.arm.UpdateArmState;

/**
 * The heart of the robot.
 *
 * <p>
 * The bulk of the robot will be implemented through various subsystems.
 * This class creates those subsystems and configures their behaviors.
 *
 * <p>
 * This class has little to do with the runtime operation. It acts more as
 * a factory to create the subsystems and write them together and to external
 * controls (both human and software). Once that has happened, the controls
 * take over and issue commands that interact with the subsystem to actually
 * do things.
 */
public class CougarRobotImpl extends CougarRobot {
  /**
   * Constructor.
   *
   * @param parameters Standard framework injected parameters.
   * @param config     Our robot's custom configuration values.
   */
  public CougarRobotImpl(CougarLibInjectedParameters parameters) {
    super(parameters);
    var logger = CougarLogger.getChildLogger(
        parameters.getRobotLogger(), "BuiltinDevices");

    // m_builtins = new BuiltinSubsystem(parameters, logger);
    m_arm = new ArmSubsystem(parameters);
    m_swerveSubsystem = new SwerveSubsystem(parameters);
    CameraServer.startAutomaticCapture();
    // m_visionSubsystem = new PhotonVisionSubsystem(parameters);
    // m_lightSubsystem = new LightSubsystem(parameters);
    m_autonChooser = new SendableChooser<Command>();
  }

  @Override
  public void robotInit() {
    AutoManager.getInstance().init(m_swerveSubsystem, m_arm);

    m_autonChooser.setDefaultOption("pathplanner auto", AutoManager.getInstance().getPathPlannerAuto(m_swerveSubsystem));
    m_autonChooser.addOption("Two Piece Auto", AutoManager.getInstance().getTwoPieceAuto(m_swerveSubsystem));

    SmartDashboard.putData(m_autonChooser);
    super.robotInit();
  }

  @Override
  public Command getAutonomousCommand() {
    CommandScheduler.getInstance().removeDefaultCommand(m_swerveSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(m_arm);
    //force the swerve subsystem to stop running the default command, setting the speed limiter should now work
    //new InstantCommand(() -> m_swerveSubsystem.stop(), m_swerveSubsystem);
    return m_autonChooser.getSelected();

    // return
    // AutoManager.getInstance().getImprovedStraightCommand(m_swerveSubsystem,
    // m_arm);
    // return AutoManager.getInstance().getRedRightGridCommand(m_swerveSubsystem,
    // m_arm);
  }

  @Override
  public void teleopInit() {
    m_swerveSubsystem.setSpeedLimiter(0.6);
    configureOperatorInterface();
    configureDriverInterface();
  }

  /**
   * Configures the driver commands and their bindings.
   */
  public void configureDriverInterface() {
    XboxController driveController = getXboxJoystick("Driver", RobotConfig.Driver.pilotPort);
    SwerveAutoBalanceYaw autoBalanceYaw = new SwerveAutoBalanceYaw(m_swerveSubsystem);

    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subsystem
    m_swerveSubsystem.setDefaultCommand(new SwerveCommand(
        m_swerveSubsystem,
        () -> -deadband(driveController.getLeftX(), 0),
        () -> -deadband(driveController.getLeftY(), 0),
        () -> -deadband(driveController.getRightX(), 0),
        () -> driveController.getYButton(),
        () -> driveController.getRightTriggerAxis(),
        () -> getMode()
        ));

    new Trigger(() -> driveController.getBButton()).onFalse(
        new InstantCommand(() -> m_swerveSubsystem.zeroGyroscope()));

    new Trigger(() -> driveController.getAButton()).whileTrue(autoBalanceYaw);

    new Trigger(() -> driveController.getXButton())
        .onTrue(new InstantCommand(() -> m_swerveSubsystem.setXModeEnabled(true)));
    new Trigger(() -> driveController.getXButton())
        .onFalse(new InstantCommand(() -> m_swerveSubsystem.setXModeEnabled(false)));
    new Trigger(() -> driveController.getPOV() == 180)
        .toggleOnTrue(new InstantCommand(() -> m_swerveSubsystem.resetOdometry()));
  }

  /**
   * Configures the operator commands and their bindings.
   */
  public void configureOperatorInterface() {
    XboxController xboxOperator = getXboxJoystick("Operator", Operator.pilotPort);

    m_arm.setDefaultCommand(new ManualArmCommand(m_arm,
        () -> -deadband(xboxOperator.getLeftY(), 0.08),
        () -> deadband(xboxOperator.getRightY(), 0.1),
        () -> xboxOperator.getLeftTriggerAxis(),
        () -> deadband(xboxOperator.getRightTriggerAxis(), 0.2),
        () -> xboxOperator.getRightBumper(),
        () -> xboxOperator.getLeftBumper()));

    // Intake tipped towards cone
    // new Trigger(() -> xboxOperator.getAButton()).onFalse(
    // new SequentialCommandGroup(
    // new UpdateArmState(GamePiece.CONE_TOWARDS),
    // new InstantCommand(() ->
    // System.out.println(StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState())),
    // new SetpointArmCommand(m_arm,
    // StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
    // true)));
    new Trigger(() -> xboxOperator.getAButton()).onFalse(
        new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))
            .andThen(
                new SetpointArmCommand(m_arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    false)));

    // Intake upright cone
    new Trigger(() -> xboxOperator.getXButton()).onFalse(
        new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_UPRIGHT))
            .andThen(
                new SetpointArmCommand(m_arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true)));

    // Intake cube
    new Trigger(() -> xboxOperator.getYButton()).onFalse(
        new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE))
            .andThen(
                new SetpointArmCommand(m_arm,
                    () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                    true)));

    // Shelf Intake
    new Trigger(() -> xboxOperator.getBButton()).onFalse(
        new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS)).andThen(
            new SetpointArmCommand(m_arm,
                () -> StateManager.getInstance().getCurrentArmGroup().getSingleShelfIntakeState(),
                false)));

    new Trigger(() -> xboxOperator.getPOV() == 180).onFalse(
        new SetpointArmCommand(m_arm, () -> ArmStateGroup.getTuck(), false));
    new Trigger(() -> xboxOperator.getPOV() == 0).onFalse(
        new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), true));
    new Trigger(() -> xboxOperator.getPOV() == 90).onFalse(
        new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getMiddleNodeState(),
            true));
    new Trigger(() -> xboxOperator.getPOV() == 270).onFalse(
        new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getLowNodeState(), false));

    // Auto High Cone Node
    // new Trigger(() -> xboxOperator.getStartButton()).onFalse(
    // new SequentialMoveArmCommand(m_arm, () ->
    // RobotConfig.ArmStates.coneHighNodeAuton, false));

    // lights
    // new Trigger(() -> xboxOperator.getStartButton()).onTrue(
    // new InstantCommand(() ->
    // StateManager.getInstance().updateLEDState(LED.YELLOW)));
    // new Trigger(() -> xboxOperator.getBackButton()).onTrue(
    // new InstantCommand(() ->
    // StateManager.getInstance().updateLEDState(LED.PURPLE)));
  }

  /**
   * Applies a deadband to the given value.
   *
   * @param value    the value to apply a deadband to
   * @param deadband the deadband to apply to the value
   * @return 0 if the value is < deadband,
   *         or value if value > deadband
   */
  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private XboxController getXboxJoystick(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
          port, role);
    }
    return new XboxController(port);
  }

  /**
   * Get controller and silence warnings if not found.
   *
   * @param role The role for the port for logging purposes.
   * @param port The expected port for the controller.
   *
   * @return controller for port, though might not be temporarily disconnected.
   */
  private PS4Controller getPS4Controller(String role, int port) {
    if (!DriverStation.isJoystickConnected(port)) {
      DriverStation.silenceJoystickConnectionWarning(true);
      CougarLogger.getAlwaysOn().warningf("No controller found on port %d for '%s'",
          port, role);
    }
    return new PS4Controller(port);
  }

  // private final BuiltinSubsystem m_builtins;
  // private final PhotonVisionSubsystem m_visionSubsystem;
  private final ArmSubsystem m_arm;
  private final SwerveSubsystem m_swerveSubsystem;
  private final SendableChooser<Command> m_autonChooser;
  // private final LightSubsystem m_lightSubsystem;
}
