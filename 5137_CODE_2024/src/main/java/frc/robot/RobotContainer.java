// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import frc.robot.Commands.*;
import frc.robot.Constants.Swerve_Constants;
import frc.robot.Subsystems.*;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class RobotContainer {

  private CommandXboxController driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;
  public LED led;

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;

  public LED_Commands led_Commands;

  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    arm = new Arm(new File(Filesystem.getDeployDirectory(), "RobotConstants.json"));
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();

    swerve_Commands = new Swerve_Commands(swerve);
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    vision.setDefaultCommand(new AddVisionMeasurement(vision, swerve));

    led = new LED();

    led_Commands = new LED_Commands();

    configureBindings();
  }



  private void configureBindings() {

    //Swerve Bindings

    
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> -MathUtil.applyDeadband(driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> MathUtil.applyDeadband(driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> !driver.leftBumper().getAsBoolean()
    ));

    driver.a()
    .onTrue(swerve_Commands.aimAtTarget());

    driver.y()
    .onTrue(swerve_Commands.zeroGyro());

    // Arm Bindings

    arm.setDefaultCommand(arm_Commands.manualMove(() -> -operator.getLeftY()));

    /*
    operator.square()
    .onTrue(arm.sysIdQuasisttatic(Direction.kForward));

    operator.triangle()
    .onTrue(arm.sysIdQuasisttatic(Direction.kReverse));

    operator.cross()
    .onTrue(arm.sysIdDynamic(Direction.kForward));

    operator.circle()
    .onTrue(arm.sysIdDynamic(Direction.kReverse));*/

    operator.circle()
    .onTrue(arm_Commands.moveToAmp());
    
    operator.square()
    .onTrue(arm_Commands.moveToSpeaker(new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return swerve.getDistanceToTarget();
      }
    }));

    operator.triangle()
    .onTrue(arm_Commands.moveToIntake());

    // Intake/Shooter Bindings

    operator.R2()
    .onTrue(intake_Commands.intakeForward())
    .onFalse(intake_Commands.stop());
    
    operator.L2()
    .onTrue(intake_Commands.intakeReverse())
    .onFalse(intake_Commands.stop());

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return intake.objectInRange();
      }
    })
    .onTrue(intake_Commands.stop());
        
    
    
    operator.cross()
    .onTrue(new ParallelCommandGroup(shooter_Commands.shoot(new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return arm.getMeasurement();
      }
    }),
    intake_Commands.intakeForward(1.5)))
    .onFalse(new ParallelCommandGroup(shooter_Commands.stop(), intake_Commands.stop()));
  }

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto("StartToMid");
  }
}