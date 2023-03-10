// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PIDDrive;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class Tank extends PIDCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private PIDDrive side;
  private PIDController controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Tank(double velocity, PIDController controller, PIDDrive side, double setAngle){
    super(controller, side::getMeasurementV, velocity, output -> side.useOutputV(output, velocity, setAngle));
    this.side = side;
    this.controller = controller;
    
    // Tolerance; 1 in/s ; 0 in/s^2
    getController().setTolerance(0, 0);
    // System.out.println("Tank:" + velocity);
    addRequirements(side);
  }
  
  // @Override
  // public void execute() {
  //   side.set(1);
  // }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("enc" + side.getMeasurement());
    return getController().atSetpoint();
  }
}
