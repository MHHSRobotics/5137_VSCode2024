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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private CommandPS4Controller driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;
  private LED led;

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;
  private LED_Commands led_Commands;

  public RobotContainer() {
    driver = new CommandPS4Controller(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    arm = new Arm(new File(Filesystem.getDeployDirectory(), "RobotConstants.json"));
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();
    led = new LED();

    swerve_Commands = new Swerve_Commands(swerve);
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    vision.setDefaultCommand(new AddVisionMeasurement(vision, swerve));
    led_Commands = new LED_Commands(led);
    led_Commands.getClass(); //Extra line to remove unused object errors

    NamedCommands.registerCommand("intake", 
        new ParallelCommandGroup(
          arm_Commands.moveToIntake(),
          intake_Commands.intakeForward()
        )
    );

    NamedCommands.registerCommand("default", arm_Commands.moveToLowered());

    NamedCommands.registerCommand("shoot",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          ),
          shooter_Commands.shootSpeaker()
        ),
        new WaitCommand(1),
        intake_Commands.intakeForward(),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
          shooter_Commands.rest(),
          intake_Commands.stop()
        )
      )
    );

    NamedCommands.registerCommand("shoot_Delay",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
              public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          ),
          shooter_Commands.shootSpeaker()
        ),
        new WaitCommand(2),
        intake_Commands.intakeForward(),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
          shooter_Commands.rest(),
          intake_Commands.stop()
        )
      )
    );

    swerve.setUpPathPlanner();

    configureBindings();
  }

  private void configureBindings() {

    //Swerve Bindings

    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> MathUtil.applyDeadband(driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> MathUtil.applyDeadband(getAllianceInvert()*driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> !driver.R2().getAsBoolean()
    ));

    driver.cross()
    .onTrue(swerve_Commands.alignToSpeaker(true));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return Math.abs(driver.getRightX()) > Swerve_Constants.RX_Deadband;
      }
    })
    .onTrue(swerve_Commands.alignToSpeaker(false));

    driver.triangle()
    .onTrue(swerve_Commands.zeroGyro());

    driver.touchpad()
    .onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    
    // Arm Bindings

    arm.setDefaultCommand(arm_Commands.manualMove(() -> -operator.getLeftY()));

    operator.triangle()
    .onTrue(
      new ParallelCommandGroup(
        arm_Commands.moveToIntake(),
        intake_Commands.intakeForward()
      )
    )
    
    .onFalse(new ParallelCommandGroup(
        arm_Commands.moveToDefault(),
        intake_Commands.stop()
      )
    );

    // Shooting Bindings

    operator.cross()

    .onTrue(
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        new WaitCommand(1),
        new ParallelCommandGroup(
          arm_Commands.moveToSpeaker(
            new DoubleSupplier() {
              @Override
                public double getAsDouble() {
                return swerve.getDistanceToTarget();
              }
            }
          )
        ),
        new WaitCommand(1),
        intake_Commands.intakeForward()
      )
    )

    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault(),
        intake_Commands.stop()
      )
    );

    operator.circle()
    .onTrue(
      new SequentialCommandGroup(
        arm_Commands.moveToAmp(),
        new WaitCommand(1),
        shooter_Commands.shootIntake()
      )
    )
    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault()
      )
    );

    // Intake Bindings & Stop Command

    //operator.R1().onTrue(swerve_Commands.driveToAmp());
    //operator.L1().onTrue(swerve_Commands.driveToTrap());
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
    .onTrue(
      new ParallelCommandGroup(
        intake_Commands.stop(),
        led_Commands.greenLedsOn()
      ))
    .onFalse(led_Commands.greenLedsOff());

    //Swerve Invert

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false);
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.getAlliance().isPresent() ? true : false);
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return (DriverStation.isEnabled());
      }
    })
    .onTrue(new InstantCommand(() -> swerve.motorInvert()));
  }
  

  public Command getAutonomousCommand() {
    return swerve_Commands.runAuto();
  }
  
  private double getAllianceInvert(){
    if(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false)
    {
      return -1;
    }
    return 1;
  }

  public void autonomousInit() {
    swerve.autonomousInit();
  }
}