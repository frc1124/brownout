// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants;
// import com.revrobotics.RelativeEncoder;

// public class PIDArm extends PIDSubsystem{

//     public final Encoder encoder;

//     public final CANSparkMax motor;
//     private final AHRS navx = new AHRS();

//     private PIDController controllerV;

//     private boolean isLeft;

//     public PIDArm(CANSparkMax motor, Encoder encoder, PIDController controllerV) {
//         super(controllerV);
//         this.controllerV = controllerV;
//         this.motor = motor;
//         this.encoder = encoder;
//         // navx.reset();
        
//         // Set the distance per pulse for the drive encoders. We can simply use the
//         // distance traveled for one rotation of the wheel divided by the encoder
//         // resolution.
//         //encoder.setAverageDepth((int) w(mod * 2 * Math.PI * Constants.WHEELRADIUS / Constants.ENCODERRESOLUTION));
//         //encoder.setDistancePerPulse((3 * Math.PI * 2) / 360); //operating frequency * 60 /maxrpm
//         encoder.reset();
//     }


//     public AHRS getNavxInstance() {
//         return navx;
//     }

//     @Override
//     protected void useOutput(double output, double setpoint) {
//         final double out = controllerV.calculate(encoder.get(), setpoint);
//         double outFiltered = MathUtil.clamp(out, -8, 8);
//         motor.setVoltage(outFiltered);
//     }
    
//     public void stop() {
//         motor.set(0);
//     }

//     public double getMeasurement() {
//         return encoder.get();
//     }
        
//     public void set(double x) {
//     motor.set(x);
//     }
    
//     public double ret() {
//         return 1.0;
//     }
    
// }
