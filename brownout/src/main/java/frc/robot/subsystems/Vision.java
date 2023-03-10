// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TankCommandGroup;
import frc.robot.commands.TankCommandGroupDistance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  NetworkTable table;
  final NetworkTableEntry angleEntry;
  final NetworkTableEntry distEntry;

  public Vision() {
    table = ntinst.getTable("Vision");
    angleEntry = table.getEntry("angle");
    distEntry = table.getEntry("distance");
  }

  public double getAngle() {
    return angleEntry.getDouble(100);
  }

  public double getDistance() {
    return distEntry.getDouble(100);
  }

  public void moveToObject() {
    CommandScheduler.getInstance().schedule(new TankCommandGroup(0, 0, getAngle(), null));
    CommandScheduler.getInstance().schedule(new TankCommandGroupDistance(40, getDistance(), null) );
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
