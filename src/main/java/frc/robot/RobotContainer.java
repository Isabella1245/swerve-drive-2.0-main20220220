package frc.robot;

import frc.robot.commands.TeleopWithJoystick;
import frc.robot.subsystems.SwerveWheelController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveWheelController m_SwerveWheelController = new SwerveWheelController();
  
    private final Joystick joystickLower = new Joystick(0);
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();

        m_SwerveWheelController.setDefaultCommand(new TeleopWithJoystick(m_SwerveWheelController, 
        joystickLower.getRawAxis(0),
        joystickLower.getRawAxis(1)*-1,
        joystickLower.getRawAxis(2), 
        joystickLower.getRawButton(1)));
    }


    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_SwerveWheelController::exampleCondition)
        //    .onTrue(new TeleopWithJoystick(m_SwerveWheelController));
        
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
      }
    
    }