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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private CommandXboxController driver;
  private CommandPS4Controller operator;

  private Swerve swerve;
  private Arm arm;
  private Intake intake; 
  private Shooter shooter;
  private Vision vision;
  private LED led;
  //private Climb climb;

  private Swerve_Commands swerve_Commands;
  private Arm_Commands arm_Commands;
  private Intake_Commands intake_Commands;
  private Shooter_Commands shooter_Commands;
  private LED_Commands led_Commands;
  //private Climb_Commands climb_Commands;

  public RobotContainer() {
    driver = new CommandXboxController(0);
    operator = new CommandPS4Controller(1);

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    arm = new Arm(new File(Filesystem.getDeployDirectory(), "RobotConstants.json"));
    intake = new Intake();
    shooter = new Shooter();
    vision = new Vision();
    led = new LED();
    //climb = new Climb();
    

    swerve_Commands = new Swerve_Commands(swerve);
    arm_Commands = new Arm_Commands(arm);
    intake_Commands = new Intake_Commands(intake);
    shooter_Commands = new Shooter_Commands(shooter);
    vision.setDefaultCommand(new AddVisionMeasurement(vision, swerve));
    led_Commands = new LED_Commands(led);
    //climb_Commands = new Climb_Commands(climb);

    NamedCommands.registerCommand("intake", 
        new ParallelCommandGroup(
          arm_Commands.moveToIntake(),
          intake_Commands.intakeForward()
        )
    );

    NamedCommands.registerCommand("default", arm_Commands.moveToStage());

    NamedCommands.registerCommand("release",
      intake_Commands.intakeForward()
  );

    NamedCommands.registerCommand("shoot",
       new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(1.1), new WaitUntilCommand(() -> {return arm.atSetpoint();})),
        intake_Commands.intakeForward()
      )
    );

     NamedCommands.registerCommand("shootKindaFast",
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(1.1), new WaitUntilCommand(() -> {return arm.atSetpoint();})),
        intake_Commands.intakeForward()
      )
    );

     NamedCommands.registerCommand("shootDelayFast",
     new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(1.3), new WaitUntilCommand(() -> {return arm.atSetpoint();})),
        intake_Commands.intakeForward()
      )
    );

    NamedCommands.registerCommand("shootFast",
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(.95), new WaitUntilCommand(() -> {return arm.atSetpoint();})),
        intake_Commands.intakeForward()
      )
    );

        NamedCommands.registerCommand("liftArm3.6",
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
            public double getAsDouble() {
              return 3.632;
            }
          }
        ));

          NamedCommands.registerCommand("liftArm4.3",
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
            public double getAsDouble() {
              return 4.4;
            }
          }
        ));

        NamedCommands.registerCommand("liftArm4.2",
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
            public double getAsDouble() {
              return 4.22;
            }
          }
        ));

    NamedCommands.registerCommand("amp",
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          arm_Commands.moveToAmp(),
          shooter_Commands.shootAmp()
        ),
        new WaitCommand(0.65),
        intake_Commands.intakeForward(),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
          shooter_Commands.rest(),
          intake_Commands.stop()
        )
      )
    );

    NamedCommands.registerCommand("shoot_Delay",
       new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(1.5), new WaitUntilCommand(() -> {return arm.atSetpoint();})),
        intake_Commands.intakeForward()
      )
    );

    swerve.setUpPathPlanner();

    configureBindings();
  }

  private void configureBindings() {

     /* 
    driver.a()
    .onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver.b()
    .onTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driver.x()
    .onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driver.y()
    .onTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    */

    //Swerve Bindings

    Trigger leftBumper = driver.leftBumper();
    
    swerve.setDefaultCommand(swerve_Commands.drive(
      () -> getAllianceInvert()*-MathUtil.applyDeadband(driver.getLeftY(), Swerve_Constants.LY_Deadband),
      () -> getAllianceInvert()*-MathUtil.applyDeadband(driver.getLeftX(), Swerve_Constants.LX_Deadband),
      () -> -MathUtil.applyDeadband(driver.getRightX(), Swerve_Constants.RX_Deadband),
      () -> true
    ));

    driver.a()
    .onTrue(swerve_Commands.alignToSpeaker(true));

    new Trigger(new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return Math.abs(driver.getRightX()) > Swerve_Constants.RX_Deadband;
      }
    })
    .onTrue(swerve_Commands.alignToSpeaker(false));

    /* 
    driver.b()
    .onTrue(swerve_Commands.driveToAmp());

    driver.x()
    .onTrue(swerve_Commands.driveToTrap());
*/
    driver.y()
    .onTrue(swerve_Commands.zeroGyro());

    // Other Bindings

    driver.back()
    .onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    
    
    /* =
    operator.cross().onTrue(arm.sysIdQuasisttatic(SysIdRoutine.Direction.kForward));
    operator.square().onTrue(arm.sysIdQuasisttatic(SysIdRoutine.Direction.kReverse));
    operator.triangle().onTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    operator.circle().onTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */

    
    // Arm Bindings
    arm.setDefaultCommand(arm_Commands.manualMove(() -> -MathUtil.applyDeadband(operator.getLeftY(), 0.1)));

    // Climb Bindings

    //climb.setDefaultCommand(climb_Commands.move(()-> -MathUtil.applyDeadband(operator.getRightY(), Climb_Constants.RY_Deadband)));

    // Shooting Bindings

    operator.cross()
    .onTrue(
      new SequentialCommandGroup(
        shooter_Commands.shootSpeaker(),
        arm_Commands.moveToSpeaker(
          new DoubleSupplier() {
            @Override
              public double getAsDouble() {
              return swerve.getDistanceToTarget();
            }
          }
        ),
        new ParallelRaceGroup(new WaitCommand(1.1), new WaitUntilCommand(() -> {return arm.atSetpoint() && shooter.atSpeed();})),
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
        shooter_Commands.shootAmp()
      )
    )
    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        arm_Commands.moveToDefault()
      )
    );

    operator.square()
    .onTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
        arm_Commands.moveToPass(),
        shooter_Commands.pass()
        ),
        new WaitCommand(0.5),
        intake_Commands.intakeForward()
      )
    )
    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        intake_Commands.stop()
      )
    );

    operator.options()
    .onTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
        arm_Commands.moveToTrap(),
        shooter_Commands.shootTrap()
        ),
        new WaitCommand(1),
        intake_Commands.intakeForward()
      )
    )
    .onFalse(
      new ParallelCommandGroup(
        shooter_Commands.stop(),
        intake_Commands.stop()
      )
    );

    operator.povDown()
    .onTrue(arm_Commands.moveToStage());

    operator.povUp()
    .onTrue(arm_Commands.moveToDefault());

    operator.R1()
    .onTrue(shooter_Commands.shootSpeaker());

    operator.L1()
    .onTrue(shooter_Commands.stop());

    // Intake Bindings & Stop Command

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
        new SequentialCommandGroup(new WaitCommand(0.1),
        intake_Commands.stop()),
        led_Commands.greenLedsOn()
      )
      )
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