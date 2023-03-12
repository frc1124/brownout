package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDDrive extends PIDSubsystem{

    public final Encoder encoder;

    public final MotorControllerGroup motors;
    private PIDController controllerD;
    private PIDController controllerV;
    private boolean isLeft;

    public PIDDrive(MotorControllerGroup motors, Encoder encoder, PIDController controllerV, PIDController controllerD, boolean isLeft) {
        super(controllerV);
        this.controllerV = controllerV;
        this.controllerD = controllerD;
        this.motors = motors;
        this.isLeft = isLeft;
        this.encoder = encoder;
//        this.kP = kP;

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        if(isLeft) {
            motors.setInverted(true);
        }

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        //encoder.setAverageDepth((int) (mod * 2 * Math.PI * Constants.WHEELRADIUS / Constants.ENCODERRESOLUTION));
        encoder.setDistancePerPulse((3 * Math.PI * 2) / 360); //operating frequency * 60 /maxrpm
        // encoder.reset();
    }

  
    @Override
    public void useOutput(double output, double setpoint) {
        final double out = controllerD.calculate(encoder.getDistance(), setpoint);
 //       double outFiltered = MathUtil.clamp(out, -8, 8);
         motors.set(out);
//        motors.setVoltage(outFiltered);
    }

    @Override
    public double getMeasurement() {
        return encoder.getDistance();
    }

    public void useOutputV(double output, double setpoint, double setAngle) {

        final double out = controllerV.calculate(encoder.getRate(), setpoint);

        if (isLeft) { 

            SmartDashboard.putNumber("Left speed", out);

        } else {

            SmartDashboard.putNumber("Right speed", out);

        }
        double outFiltered = MathUtil.clamp(out, -8, 8);
        motors.setVoltage(outFiltered);
        //navx.zeroYaw();
    }
    public void useOutputAuto(double speed, double distance) {
        final double out = speed;
         double outFiltered = MathUtil.clamp(out, -8, 8);
        motors.setVoltage(outFiltered);
    }

    public void stop() {
        motors.set(0);
    }

    public double getMeasurementV() {
        return encoder.getRate();
    }
        
    public void set(double x) {
    motors.set(x);
    }
    
    public double ret() {
        return 1.0;
    }
    
}
