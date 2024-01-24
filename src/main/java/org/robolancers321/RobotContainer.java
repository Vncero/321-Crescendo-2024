package org.robolancers321;

import org.robolancers321.commands.drivetrain.TeleopDrive;
import org.robolancers321.commands.drivetrain.TuneDriveToTarget;
import org.robolancers321.subsystems.drivetrain.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Drivetrain drivetrain;

  XboxController controller;

  SendableChooser<Command> autoChooser;

  public RobotContainer() {
    this.drivetrain = Drivetrain.getInstance();

    this.controller = new XboxController(0);

    this.autoChooser = AutoBuilder.buildAutoChooser();

    this.configureBindings();
    this.configureAutoChooser();
  }

  private void configureBindings() {
    this.drivetrain.setDefaultCommand(new TeleopDrive(this.controller));

    new Trigger(this.controller::getXButton).whileTrue(new TuneDriveToTarget(0.0, 1.0, 0.0));
    
    new Trigger(this.controller::getAButton).onTrue(new InstantCommand(this.drivetrain::zeroYaw));
  }

  private void configureAutoChooser(){
    NamedCommands.registerCommand("Say Hello", new PrintCommand("Hello"));

    this.autoChooser.addOption("Do Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }
}
