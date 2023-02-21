package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;


import frc.robot.subsystems.SwerveWheelController;

public class TeleopWithJoystick extends CommandBase {

    //Joystick joystickLower = new Joystick(0);

    private SwerveWheelController swerveJoystick = null;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;

    private boolean fieldOriented = false;


  /** Creates a new DriveWithJoysticks. */
  public TeleopWithJoystick(SwerveWheelController swerveJoystick, double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {

    this.swerveJoystick = swerveJoystick;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zSpeed = zSpeed;
    this.fieldOriented = fieldOriented;

    addRequirements(swerveJoystick);

  }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {

    swerveJoystick.resetGyro();
   }
 
   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {

    //ReadJoystick();

     double m_xSpeed = xSpeed;
     double m_ySpeed = ySpeed;
     double m_zSpeed = zSpeed;
 
     //apply deadband
     m_xSpeed = Math.abs(m_xSpeed) > 0.15 ? m_xSpeed : 0;
     m_ySpeed = Math.abs(m_ySpeed) > 0.15 ? m_ySpeed : 0;
     m_zSpeed = Math.abs(m_zSpeed) > 0.15 ? m_zSpeed : 0;
 
     //set chassis speeds
     if(fieldOriented){
        fieldOriented = !fieldOriented;
        swerveJoystick.setFOD(fieldOriented);
     } 
     
     swerveJoystick.drive(m_xSpeed, m_ySpeed, m_zSpeed, swerveJoystick.gyroAngle());
   } 
 
 
   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    swerveJoystick.stop();
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }

   /*
   public void ReadJoystick() {
    xSpeed = joystickLower.getRawAxis(0);
    ySpeed = joystickLower.getRawAxis(1) * -1;
    zSpeed = joystickLower.getRawAxis(2);

    fieldOriented = joystickLower.getRawButton(1);
   }
   */
 }