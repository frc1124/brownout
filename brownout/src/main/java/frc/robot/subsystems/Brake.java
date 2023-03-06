// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Brake extends SubsystemBase {
//     private static Servo leftServo;
//     private static Servo rightServo;

//     private static CANSparkMax leftBrake;
//     private static CANSparkMax rightBrake;
    
     
//     public Brake() {
//         leftServo = new Servo(Constants.LEFT_SERVO);
//         rightServo = new Servo(Constants.RIGHT_SERVO);

//         leftBrake = new CANSparkMax(Constants.LEFT_BRAKE, MotorType.kBrushless);
//         rightBrake = new CANSparkMax(Constants.RIGHT_BRAKE, MotorType.kBrushless);
//     }

//     public void Stop() {
//         rightBrake.set(2);
//         leftBrake.set(2);
//     }

//     public void Go() {
//         leftServo.setAngle(180);
//         rightServo.setAngle(180);

//         rightBrake.set(-2);
//         leftBrake.set(-2);
//     }
// }
