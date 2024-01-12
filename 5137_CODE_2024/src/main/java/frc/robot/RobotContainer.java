// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  public static Joystick driver;

  public static Arm arm;

  public static Arm_Commands arm_Commands;

  public RobotContainer() {

    driver = new Joystick(0);
    arm = new Arm();

    arm_Commands = new Arm_Commands(arm);

    configureBindings();
  }

  private void configureBindings() {
    new Trigger(createBooleanSupplier(driver, 4, 3))
    .onTrue(arm_Commands.moveForward())
    .onFalse(arm_Commands.stopMoving());

    new Trigger(createBooleanSupplier(driver, 3, 4))
    .onTrue(arm_Commands.moveBackward())
    .onFalse(arm_Commands.stopMoving());
  }

  public static BooleanSupplier createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort) {
    BooleanSupplier booleanSupply;
    booleanSupply = () -> {
      if (controller != null) {
        if (controller.getRawAxis(requiredPort) > 0.1 && controller.getRawAxis(dependentPort) < 0.1) {
          return true;
        } else {
          return false;
        }
      } else {return false;}
    };
    return booleanSupply;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
