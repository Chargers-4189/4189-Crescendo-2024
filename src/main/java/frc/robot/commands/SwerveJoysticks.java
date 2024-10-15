// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveJoysticks extends PIDCommand {
  /** Creates a new SwerveJoysticks. */
  public SwerveJoysticks(DriveSubsystem swerve, Joystick leftStick, Joystick rightStick) {
    super(
        // The controller that the command will use
        new PIDController(.02, 0,.000),
        // This should return the measurement
        () -> swerve.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> swerve.getChosenAngle(),
        // This uses the output
        output -> {
          double joystickXValue = leftStick.getX();
          double joystickYValue = leftStick.getY();
          double xSpeed = MathUtil.applyDeadband(joystickYValue*Math.abs(joystickYValue), OIConstants.kDriveDeadband);
          double ySpeed = MathUtil.applyDeadband(joystickXValue*Math.abs(joystickXValue), OIConstants.kDriveDeadband);
          double rotationSpeed = MathUtil.applyDeadband(-rightStick.getX(), OIConstants.kDriveDeadband);
          /*

          if(Math.abs(rotationSpeed)> 0.1){        
            swerve.setChosenAngle(Math.atan2(-rightStick.getY(), rightStick.getX()) * 360 / (2*Math.PI));
          } 
          
          //.println("GET Y: " + -rightStick.getY() + " GET X: " + rightStick.getX());
          System.out.println(Math.atan2(-rightStick.getY(), rightStick.getX()) * 360 / (2*Math.PI));
          
          // Use the output here
          swerve.drive(xSpeed, ySpeed, -output, true, true);
          SmartDashboard.putNumber("Angle", swerve.getAngle());
          SmartDashboard.putNumber("Chossen Angle", swerve.getChosenAngle());
          */
          
          if(xSpeed == 0 && ySpeed==0 && rotationSpeed == 0){
            swerve.setX();
          } else {
            if(xSpeed > 0){
                swerve.drive( 1.0 * xSpeed, 1.0 * ySpeed, 0.8 * rotationSpeed - .005, true, true);
            } else {
              swerve.drive( 1.0 * xSpeed, 1.0 * ySpeed, 0.8 * rotationSpeed + .005, true, true);
            }
          }
          
        });
        //absolute angle is tan inverse
        addRequirements(swerve);
    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
