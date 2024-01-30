// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
public class RobotContainer {

  public static CommandPS4Controller driver;
  public static CommandPS4Controller operator;

  public static Arm arm;

  public static Arm_Commands arm_Commands;

  public RobotContainer() {

    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);
    arm = new Arm();

    arm_Commands = new Arm_Commands(arm);

    configureBindings();
  }



  private void configureBindings() {
    operator.square()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.toStop());
    operator.circle()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.toStop());
    operator.cross()
    .onTrue(intake_Commands.continuousIntake());
    operator.R1()
    .onTrue(intake_Commands.shootDefault())
    .onFalse(intake_Commands.stop());
    operator.L1()
    .onTrue(intake_Commands.launch());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}