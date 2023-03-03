// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.kauailabs.navx.frc.Tracer;

import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics  extends SubsystemBase {

  Solenoid solenoidL;
  Solenoid solenoidR;
  Compressor compressor;
  
  /** Creates a new ExampleSubsystem. */
  public Pneumatics() {
    solenoidL = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    solenoidR = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */


  public void enableComp() {
    compressor.enableDigital();
  }

  public void disableComp() {
    compressor.disable();
  }

  public void solFwd() {
    solenoidL.set(true);
    solenoidR.set(true);
  }

  public void solBack() {
    solenoidL.set(false);
    solenoidR.set(false);
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
