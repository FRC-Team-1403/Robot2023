// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  NetworkTableInstance inst;
  NetworkTable table;
  BooleanSubscriber sub;
  DoubleSubscriber sub2;

  static void func(NetworkTableEvent event) {
    System.out.println(event.logMessage.message);
  }

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    inst = NetworkTableInstance.getDefault();
    inst.startServer();
    table = inst.getTable("conetable");
    sub = table.getBooleanTopic("presentcone").subscribe(false);
    sub2 = table.getDoubleTopic("xpos").subscribe(0.0);
    inst.addLogger(0, 10, ExampleSubsystem::func);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.print("present cone: ");
    System.out.println(sub.get());
    System.out.print("value: ");
    System.out.println(sub2.get());
    System.out.print("connected: ");
    System.out.println(inst.isConnected());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
