// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {

  private final CommandPS4Controller driver;
  private final CommandPS4Controller operator;

  private final Arm arm;
  private final Vision vision; 

  private final Arm_Commands arm_Commands;

  public RobotContainer() {

    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);
    arm = new Arm();
    vision = new Vision();

    arm_Commands = new Arm_Commands(arm);
    vision.setDefaultCommand(new AddVisionMeasurement(vision));

    configureBindings();
  }

  private void configureBindings() {
    // Operator Bindings
    operator.square()
    .onTrue(arm_Commands.moveToIntake());

    operator.triangle()
    .onTrue(arm_Commands.moveToStart());

    operator.circle()
    .onTrue(arm_Commands.moveToAmp());

    operator.cross()
    .onTrue(arm_Commands.moveToSpeaker());

    operator.L2()
    .onTrue(arm_Commands.moveBackward())
    .onFalse(arm_Commands.stopMoving());

    operator.R2()
    .onTrue(arm_Commands.moveForward())
    .onFalse(arm_Commands.stopMoving());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
