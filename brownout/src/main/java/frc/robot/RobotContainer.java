// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.ArmExtend;
import frc.robot.commands.ArmRaise;
import frc.robot.commands.ArmLower;
import frc.robot.commands.Autos;
import frc.robot.commands.BrakeDisable;
import frc.robot.commands.BrakeStop;
import frc.robot.commands.BrakeOpen;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.EnableVision;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.PIDArm;
import frc.robot.commands.TankCommandGroup;
import frc.robot.subsystems.PIDDrive;
//import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Stabilizer;
//import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Claw;

import java.util.HashMap;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.CounterBase;
import com.kauailabs.navx.frc.AHRS;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController j;
  // The robot's subsystems and commands are defined here...

  //motors
  public final CANSparkMax leftLeader = new CANSparkMax(Constants.LEFTFRONT, MotorType.kBrushless);
  public final CANSparkMax leftFollower = new CANSparkMax(Constants.LEFTBACK, MotorType.kBrushless);
  public final CANSparkMax rightLeader = new CANSparkMax(Constants.RIGHTFRONT, MotorType.kBrushless);
  public final CANSparkMax rightFollower = new CANSparkMax(Constants.RIGHTBACK, MotorType.kBrushless);

  //encoders
  public final Encoder leftEncoder = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
  public final Encoder rightEncoder = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);

  //motorcontrollers
  public final MotorControllerGroup lefts = new MotorControllerGroup(leftLeader, leftFollower);
  public final MotorControllerGroup rights = new MotorControllerGroup(rightLeader, rightFollower);

  //pidcontrollers
  public final PIDController leftDController = new PIDController(
    Constants.DIST_L_P, Constants.DIST_L_I, Constants.DIST_L_D);
  public final PIDController rightDController = new PIDController(
    Constants.DIST_R_P, Constants.DIST_R_I, Constants.DIST_R_D);
  public final PIDController leftVController = new PIDController(
    Constants.VEL_L_P, Constants.VEL_L_I, Constants.VEL_L_D);
  public final PIDController rightVController = new PIDController(
    Constants.VEL_R_P, Constants.VEL_R_I, Constants.VEL_R_D);
  //public final PIDArm armController = new PIDArm(leftFollower, leftEncoder, leftDController)

  public final AHRS navx = new AHRS();
  public final PIDDrive left = new PIDDrive(lefts, leftEncoder, leftVController, leftDController, false, Constants.ANGLE_L_P, navx);
  public final PIDDrive right = new PIDDrive(rights, rightEncoder, rightVController, rightDController, true, Constants.ANGLE_R_P, navx);

  // Pneumatics
  //public Pneumatics pneumatics = new Pneumatics();

  public Stabilizer stabilizer = new Stabilizer(navx);
  
  
  // Arm deez nuts
  public CANSparkMax armMotor = new CANSparkMax(Constants.ARM_ID, MotorType.kBrushless);
  public AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);
  public PIDController controllerD = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
  public Arm arm = new Arm(armMotor, pot, controllerD);

  // Claw
  public Claw claw = new Claw();

  // Brake 
  public Brake brake = new Brake();

  // Autonomous Commands
  public SendableChooser<Command> chooser = new SendableChooser<>(); 

  // Vision
  //Vision vision = new Vision();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    j = new XboxController(0);
    // Configure the trigger bindings
    configureBindings();
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  public static HashMap<String, JoystickButton> logitechMap = new HashMap<String, JoystickButton>();

  public static JoystickButton getKey(String key) { 

    logitechMap.put("1", new JoystickButton(j, 1));
    logitechMap.put("2", new JoystickButton(j, 2));
    logitechMap.put("3", new JoystickButton(j, 3));
    logitechMap.put("4", new JoystickButton(j, 4));
    logitechMap.put("left button", new JoystickButton(j, 5));
    logitechMap.put("right button", new JoystickButton(j, 6));
    logitechMap.put("left trigger", new JoystickButton(j, 7));
    logitechMap.put("right trigger", new JoystickButton(j, 8));
    logitechMap.put("9", new JoystickButton(j, 9));
    logitechMap.put("10", new JoystickButton(j, 10));
    logitechMap.put("left joystick", new JoystickButton(j, 11));
    logitechMap.put("right joystick", new JoystickButton(j, 12));

    return logitechMap.get(key);
  }
  
  private void configureBindings() {
    // getKey("botton right").onTrue(new ArmExtend(30, controllerD, null)); 
    // getKey("thumb button").onTrue(new ArmExtend(50, controllerD, null)); 
    // getKey("botton left").onTrue(new ArmExtend(0, controllerD, null)); 
    getKey("right trigger").whileTrue(new ArmRaise(arm)); 
    getKey("left trigger").whileTrue(new ArmLower(arm)); 
    getKey("left button").whileTrue(new ClawClose(claw));
    getKey("right button").whileTrue(new ClawOpen(claw));
    getKey("9").whileTrue(new BrakeOpen(brake));
    getKey("10").toggleOnTrue(new BrakeStop(brake));
    //getKey("10").toggleOnFalse(new SolenoidBack(pneumatics));
    getKey("4").onTrue(new BrakeDisable(brake));


  }

  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    return chooser.getSelected();
  }
  
}
