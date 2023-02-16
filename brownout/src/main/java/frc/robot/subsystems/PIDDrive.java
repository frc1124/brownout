package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PIDDrive extends PIDSubsystem{

    public final RelativeEncoder encoder;

    public final MotorControllerGroup motors;
    private final AHRS navx = new AHRS();

    private PIDController controllerD;
    private PIDController controllerV;

    private boolean isLeft;

    public PIDDrive(MotorControllerGroup motors, RelativeEncoder encoder, PIDController controllerV, PIDController controllerD, boolean isLeft) {
        super(controllerV);
        this.controllerV = controllerV;
        this.controllerD = controllerD;
        this.motors = motors;
        this.isLeft = isLeft;
        this.encoder = encoder;
        // navx.reset();

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        int mod = -1;
        if(isLeft) {
            mod = 1;
            motors.setInverted(true);
        }

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        //encoder.setAverageDepth((int) (mod * 2 * Math.PI * Constants.WHEELRADIUS / Constants.ENCODERRESOLUTION));
        encoder.setAverageDepth(8);
        // encoder.reset();
    }

    public AHRS getNavxInstance() {
        return navx;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        final double out = controllerD.calculate(encoder.getPosition(), setpoint);
        double outFiltered = MathUtil.clamp(out, -8, 8);
        motors.setVoltage(outFiltered);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getPosition();
    }

    public void useOutputV(double output, double setpoint) {
        final double out = controllerV.calculate(encoder.getVelocity(), setpoint);
        double outFiltered = MathUtil.clamp(out, -8, 8);
        motors.setVoltage(outFiltered);
    }

    public void stop() {
        motors.set(0);
    }

    public double getMeasurementV() {
    return encoder.getVelocity();
    }
        
    public void set(double x) {
    motors.set(x);
    }
    
    public double ret() {
        return 1.0;
    }
    
}
