// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  public CommandPS4Controller driver;
  public CommandPS4Controller operator;

  public Swerve swerve;

  public Swerve_Commands swerve_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(driver, new File(Filesystem.getDeployDirectory(),"swerve"));

    swerve_Commands = new Swerve_Commands(swerve);

    configureBindings();
  }

  private void configureBindings() {
    driver.L1()
    .onTrue(swerve_Commands.disableFieldRelative())
    .onFalse(swerve_Commands.enableFieldRelative());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
