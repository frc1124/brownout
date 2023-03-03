// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.security.spec.EncodedKeySpec;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import frc.robot.Constants;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

// public class Arm extends TrapezoidProfileSubsystem {
//     public final CANSparkMax winch;
//     public final Encoder motorEncoder;
//     public final ArmFeedforward feedforward = new ArmFeedforward(getEncoderOffset(), getEncoderOffset(), getEncoderOffset());

//     /** Creates a new ExampleSubsystem. */
//     public Arm(CANSparkMax winch, Encoder motorEncoder) {
//         super(
//             new TrapezoidProfile.Constraints(
//                 , 
//                 )
//         );
//         //winch. set pids somehow


//         this.winch = winch;
//         this.motorEncoder = motorEncoder; 

//         //addRequirements(winch);
//     }

//     @Override
//     public void useState(TrapezoidProfile.State setpoint) {
//         // Calculate the feedforward from the sepoint
//         double feedforward = feedforward.calculate(setpoint.position, setpoint.velocity);
//         // Add the feedforward to the PID output to get the motor output
//         winch.setSetpoint(
//             ExampleSmartMotorController.PIDMode.kPosition, setpoint.position, feedforward / 12.0);
//     }

//     public Command setArmGoalCommand(double kArmOffsetRads) {
//         return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
//     }


//     public void extend() {
//         winch.set(1);

//     }

//     public void retract() {
//         winch.set(-1);
//     }

//     public double getEncoderOffset() {
//         return motorEncoder.getDistance();
//     }
// }
