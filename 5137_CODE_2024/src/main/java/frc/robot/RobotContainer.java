// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {

  private CommandPS4Controller driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;
  

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    arm = new Arm();
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();

    swerve_Commands = new Swerve_Commands(swerve);
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    vision.setDefaultCommand(new AddVisionMeasurement(vision));

    configureBindings();
  }



  private void configureBindings() {

    //Swerve Bindings

    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> driver.getLeftX(),
      () -> -driver.getLeftY(),
      () -> driver.getRightX(),
      () -> !driver.L1().getAsBoolean()
    ));

    driver.triangle()
    .onTrue(swerve_Commands.zeroGyro());

    // Arm Bindings

    arm.setDefaultCommand(arm_Commands.manualMove(() -> operator.getLeftX()));

    // Intake/Shooter Bindings

    operator.R2()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.stop());

    operator.L2()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.stop());

    operator.touchpad()
    .onTrue(new ParallelCommandGroup(shooter_Commands.shoot(arm.getMeasurement()), intake_Commands.intakeForward(1.0)))
    .onFalse(new ParallelCommandGroup(shooter_Commands.stop(), intake_Commands.stop()));
  }

  public Command disableInit() {
    return arm_Commands.release();
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto("StartToMid");
  }
}