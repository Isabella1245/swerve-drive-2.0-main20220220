package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveWheel;
import frc.robot.subsystems.SwerveWheelController;

import edu.wpi.first.wpilibj.AnalogInput;
//i think we can use the wpilib import for joysticks instead of the vikings' import for controller
//import viking.Controller;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  
  //public static Controller driver = new Controller(0);
  //public static Joystick driver = new Joystick(1);

	//private static CommandScheduler scheduler = CommandScheduler.getInstance();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //SwerveWheelController.getInstance();

    //put the schedule instance on the smart dashboard when the robot initializes
    //SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    System.out.println("Disabled");
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous");
  }

  @Override
  public void autonomousPeriodic() {
    //scheduler.run();
  }

  @Override
  public void teleopInit() {
    System.out.println("Teleop");
    
  }

  @Override
  public void teleopPeriodic() {
    //scheduler.run();
  }

}
