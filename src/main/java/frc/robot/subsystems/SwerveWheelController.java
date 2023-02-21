package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.TeleopWithJoystick;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveWheel;

import com.kauailabs.navx.frc.AHRS;

public class SwerveWheelController extends SubsystemBase implements Constants  {
    
    private static SwerveWheelController instance = null;
    
    //swerve wheels
    private SwerveWheel frontRight = null;
    private SwerveWheel frontLeft = null;
    private SwerveWheel backRight = null;
    private SwerveWheel backLeft = null;

    private AHRS gyro = null;

    // Get distance between wheels
    private double r = Math.sqrt((L * L) + (W * W));

    private boolean isFieldCentric = true;
    private boolean gyroEnabled = false;

    public SwerveWheelController(){
    
        //potential robot container content
        frontRight = new SwerveWheel(FRDid, FRTid, FRTencoderID, FRTencoderOffset, "Front Right", true, true);
        frontLeft = new SwerveWheel(FLDid, FLTid, FLTencoderID, FLTencoderOffset, "Front Left", true, true);
        backRight = new SwerveWheel(BRDid, BRTid, BRTencoderID, BRTencoderOffset, "Back Right", true, true);
        backLeft = new SwerveWheel(BLDid, BLTid, BLTencoderID, BLTencoderOffset, "Back Left", true, true);
        

        try {
            gyro = new AHRS(SPI.Port.kMXP); 
            gyroEnabled = true;
        } catch (RuntimeException ex ) {
            System.out.println("--------------");
            System.out.println("NavX not plugged in");
            System.out.println("--------------");
            gyroEnabled = false;
        }

        frontRight.enable();
        frontLeft.enable();
        backRight.enable();
        backLeft.enable();
    }

         // x = strafe, y = speed, z = rotation 
    // Holonomic drive
    public void drive(double x, double y, double z, double gyroValue) {

        //calculate magnitude of joystick
        // Calculate magnitude of joystick

        y *= -1;
        double magnitude = Math.sqrt((Math.pow(x, 2)) + (Math.pow(y,2)));
        double frontLeftSpeed = 0;
        double frontRightSpeed = 0;
        double backRightSpeed = 0;
        double backLeftSpeed = 0;
   
        if (magnitude >= 0.15) {

            // I got this bit of code from the NavX website
            if (isFieldCentric == true && gyroEnabled == true) {
                // Convert gyro angle to radians
                double gyro = gyroValue * Math.PI / 180;

                //field orientation math
                double temp = x * Math.cos(gyro) + y * Math.sin(gyro); 
                y = -x * Math.sin(gyro) + y * Math.cos(gyro); 
                x = temp;
            }

            // -------------------------------------
            // Do the swerve wheel math for speed and angles
            // -------------------------------------
            double a = x - z * (L / r);
            double b = x + z * (L / r);
            double c = y - z * (W / r);
            double d = y + z * (W / r);

            frontLeftSpeed = Math.sqrt((b * b) + (c * c));
            frontRightSpeed = Math.sqrt((b * b) + (d * d));
            backRightSpeed = Math.sqrt((a * a) + (d * d));
            backLeftSpeed = Math.sqrt((a * a) + (c * c));

            double backRightAngle = Math.atan2(a, d) * 180 / Math.PI;
            double backLeftAngle = Math.atan2(a, c) * 180 / Math.PI;
            double frontRightAngle = Math.atan2(b, d) * 180 / Math.PI;
            double frontLeftAngle = Math.atan2(b, c) * 180 / Math.PI ;      

            // This bit of code normalizes the speed
            double max = frontLeftSpeed;
            max = Math.max(max, frontRightSpeed);
            max = Math.max(max, backRightSpeed);
            max = Math.max(max, backLeftSpeed);

            if (max > 1) {
                frontLeftSpeed /= max;
                frontRightSpeed /= max;
                backRightSpeed /= max;
                backLeftSpeed /= max;
            }

            frontRight.setSetpoint(frontRightAngle);
            frontLeft.setSetpoint(frontLeftAngle);
            backRight.setSetpoint(backRightAngle);
            backLeft.setSetpoint(backLeftAngle);

            frontLeft.setSpeed(frontLeftSpeed);
            frontRight.setSpeed(frontRightSpeed);
            backRight.setSpeed(backRightSpeed);
            backLeft.setSpeed(backLeftSpeed);
        } 
        else {
            frontLeft.setSpeed(0);
            frontRight.setSpeed(0);
            backRight.setSpeed(0);
            backLeft.setSpeed(0);
        }

        SmartDashboard.putNumber("magnitude", magnitude);
        SmartDashboard.putNumber("fr angle", frontRight.getSetpoint());
        SmartDashboard.putNumber("fl angle", frontLeft.getSetpoint());
        SmartDashboard.putNumber("br angle", backRight.getSetpoint());
        SmartDashboard.putNumber("bl angle", backLeft.getSetpoint());
        SmartDashboard.putNumber("gyro", gyroValue);
        SmartDashboard.putNumber("fl speed", frontLeftSpeed);
        SmartDashboard.putNumber("fr speed", frontRightSpeed);
        SmartDashboard.putNumber("Bl speed", backLeftSpeed);
        SmartDashboard.putNumber("br speed", backRightSpeed);

        //front right
        SmartDashboard.putNumber("FR AbsEnc Value", frontRight.getAbsEncValue());
        SmartDashboard.putNumber("FR AbsEnc Angle ", frontRight.getAbsAngleDeg());
        SmartDashboard.putNumber("FR QuadEnc Zero Setpoint", frontRight.getQuadEncZeroSetpoint());
        SmartDashboard.putNumber("FR QuadEnc Actual Value", frontRight.getTicks());
        SmartDashboard.putNumber("FR QuadEnc angle (ticks to angle)", frontRight.getMeasurement());
        SmartDashboard.putNumber("FR Turn Motor speed", frontRight.getTurnMotorSpeed());
        SmartDashboard.putNumber("p values", frontRight.getPValues());
        SmartDashboard.putNumber("i values", frontRight.getIValues());
        SmartDashboard.putNumber("d values", frontRight.getDValues());

        SmartDashboard.putNumber("position tolerance", frontRight.getTolerance());
        SmartDashboard.putNumber("position error", frontRight.getError());
        SmartDashboard.putNumber("calculation", frontRight.getCalculation());

        //front left
        SmartDashboard.putNumber("FL AbsEnc Value", frontLeft.getAbsEncValue());
        SmartDashboard.putNumber("FL AbsEnc Angle ", frontLeft.getAbsAngleDeg());
        SmartDashboard.putNumber("FL QuadEnc Zero Setpoint", frontLeft.getQuadEncZeroSetpoint());
        SmartDashboard.putNumber("FL QuadEnc Actual Value", frontLeft.getTicks());
        SmartDashboard.putNumber("FL QuadEnc Angle", frontLeft.getMeasurement());
        SmartDashboard.putNumber("FL Turn Motor speed", frontLeft.getTurnMotorSpeed());

        //back right
        SmartDashboard.putNumber("BR AbsEnc Value", backRight.getAbsEncValue());
        SmartDashboard.putNumber("BR AbsEnc Angle ", backRight.getAbsAngleDeg());
        SmartDashboard.putNumber("BR QuadEnc Zero Setpoint", backRight.getQuadEncZeroSetpoint());
        SmartDashboard.putNumber("BR QuadEnc Actual Value", backRight.getTicks());
        SmartDashboard.putNumber("BR QuadEnc Angle", backRight.getMeasurement());
        SmartDashboard.putNumber("BR Turn Motor speed", backRight.getTurnMotorSpeed());

        //back left
        SmartDashboard.putNumber("BL AbsEnc Value", backLeft.getAbsEncValue());
        SmartDashboard.putNumber("BL AbsEnc Angle ", backLeft.getAbsAngleDeg());
        SmartDashboard.putNumber("BL QuadEnc Zero Setpoint", backLeft.getQuadEncZeroSetpoint());
        SmartDashboard.putNumber("BL QuadEnc Actual Value", backLeft.getTicks());
        SmartDashboard.putNumber("BL QuadEnc Angle", backLeft.getMeasurement());
        SmartDashboard.putNumber("BL Turn Motor speed", backLeft.getTurnMotorSpeed());
        //SmartDashboard.putNumber("ticks to angle", frontRight.ticksToAngle());


    }

    // Zero the Gryo
    public void resetGyro() {
        gyro.reset();
    }

    // Get the Gyro Angle (-180 to 180)
    public double gyroAngle() {
        return gyro.getYaw();
    }

    // Set the controller to be field oriented drive
    public void setFOD(boolean value) {
        isFieldCentric = value;
    }

    // Get the current FOD mode
    public boolean getFOD() {
        return isFieldCentric;
    }

        public void stop(){
        frontLeft.setSpeed(0);
        frontRight.setSpeed(0);
        backRight.setSpeed(0);
        backLeft.setSpeed(0);
    }

}