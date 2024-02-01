// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  public static CommandPS4Controller driver;
  public static CommandPS4Controller operator;

  private Swerve swerve;

  private Swerve_Commands swerve_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));

    swerve_Commands = new Swerve_Commands(swerve);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> driver.getLeftX(),
      () -> -driver.getLeftY(),
      () -> -driver.getRightX(),
      () -> !driver.L1().getAsBoolean()
    ));

    
    //driver.cross()
    //.onTrue(swerve_Commands.aimAtTarget());

  }

  public Command getAutonomousCommand() {
    //return swerve_Commands.runAuto("StartToMid");
    return swerve_Commands.runAuto("StartToMid");
  }
}