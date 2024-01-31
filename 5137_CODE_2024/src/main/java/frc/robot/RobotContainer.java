// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.*;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;
<<<<<<< HEAD
=======

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
<<<<<<< HEAD

  public static CommandPS4Controller driver;
  public static CommandPS4Controller operator;

  public static Arm arm;

  public static Arm_Commands arm_Commands;
=======
  public static CommandPS4Controller driver;
  public static CommandPS4Controller operator;

  private Swerve swerve;

  private Swerve_Commands swerve_Commands;
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);
<<<<<<< HEAD
    arm = new Arm();

    arm_Commands = new Arm_Commands(arm);

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
=======

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));

    swerve_Commands = new Swerve_Commands(swerve);

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> driver.getLeftX(),
      () -> -driver.getLeftY(),
      () -> driver.getRightX(),
      () -> !driver.L1().getAsBoolean()
    ));
>>>>>>> 2da6360f20fe863bc220c7b6615b8312bdd0cb09
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto("StartToMid");
  }
}
