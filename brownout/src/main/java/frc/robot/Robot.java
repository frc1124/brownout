// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SolenoidFwd;
import frc.robot.commands.TankCommandGroup;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.commands.CompOff;
import frc.robot.commands.CompOn;
import javax.print.attribute.standard.PresentationDirection;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCMD;
  AnalogPotentiometer pressureTransducer;
  public static RobotContainer rc;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    rc = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    PneumaticsControlModule pcm = new PneumaticsControlModule();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    double scale = 250, offset = -25;
    //pressureTransducer = new AnalogPotentiometer(/* the AnalogIn port*/ 0, scale, offset);
    //rc.pneumatics.solOff();
    //rc.pneumatics.enableComp();
    //SmartDashboard.putData("Sol forward", new SolenoidFwd(rc.pneumatics));
    //SmartDashboard.putData("Sol back", new SolenoidFwd(rc.pneumatics));
    //SmartDashboard.putData("Compressor On", new CompOn(rc.pneumatics));
    //SmartDashboard.putData("Compressor Off", new CompOff(rc                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           .pneumatics));

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("PSI", pressureTransducer.get());

    double velL = Math.pow(rc.j.getLeftY(),3); // Speed given by encoder is RPM
    double velR = Math.pow(rc.j.getRightY(),3);

    SmartDashboard.putNumber("Joystick Y", velL);
    
    //double spin = rc.j.getX();
    //double twist = rc.j.getTwist();

    /* 
    double vel2 = vel;
    if (spin < -0.3) {
      if (vel < 200000) {
        vel+=200000;
      } 
      vel = -vel;
    } else if(spin > 0.3) {
      if (vel < 200000) {
        vel+=200000;
      }
      vel2 = vel;
      vel = -vel;
    }
    */

    CommandScheduler.getInstance().schedule(new TankCommandGroup(
      1000000, 
      100000, 
      rc
    ));  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
