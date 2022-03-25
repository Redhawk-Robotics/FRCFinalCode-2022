// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
//import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DMASample.DMAReadStatus;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
 
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  PS4Controller controller = new PS4Controller(0);
  PS4Controller controller2 = new PS4Controller(1);
 
 
  private CANSparkMax driveMotor = new CANSparkMax(1, MotorType.kBrushless); //
  private CANSparkMax driveMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax driveMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax driveMotor4 = new CANSparkMax(4, MotorType.kBrushless);
 
  private CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax intakeMotor2 = new CANSparkMax(6,MotorType.kBrushless);
 
  private CANSparkMax uptake = new CANSparkMax(8, MotorType.kBrushless);
 
  private CANSparkMax outtake = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax outtake2 = new CANSparkMax(9, MotorType.kBrushless);
 
  private final MotorControllerGroup leftSpeedGroup = new MotorControllerGroup(driveMotor,driveMotor2);
  private final MotorControllerGroup rightSpeedGroup = new MotorControllerGroup(driveMotor3, driveMotor4);
 
  private final MotorControllerGroup intakeGroup = new MotorControllerGroup(intakeMotor, intakeMotor2);
 
  private final MotorControllerGroup outtakeGroup = new MotorControllerGroup(outtake, outtake2);
 
 
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftSpeedGroup, rightSpeedGroup);
 
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
 
  private final DoubleSolenoid dropIntake1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final DoubleSolenoid dropIntake2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
 
 
  private final Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
   
  private double startTime = Timer.getFPGATimestamp();

  private final Command simpleAuto = new SimpleAuto(startTime, outtake, outtake2, uptake, leftSpeedGroup, rightSpeedGroup, robotDrive);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Command m_autonomousCommand;
 
 
  @Override
  public void robotInit() {

     CameraServer.startAutomaticCapture();

     m_chooser.setDefaultOption("Simple Auto", simpleAuto);

     SmartDashboard.putData(m_chooser);

 
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor2.setIdleMode(IdleMode.kBrake);
    driveMotor3.setIdleMode(IdleMode.kBrake);
    driveMotor4.setIdleMode(IdleMode.kBrake);
 
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor2.setIdleMode(IdleMode.kBrake);
 
    outtake.setIdleMode(IdleMode.kBrake);
    outtake2.setIdleMode(IdleMode.kBrake);
 
 
    uptake.setIdleMode(IdleMode.kBrake);
 
    shifter.set(false);
 
    uptake.setInverted(true);
 
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
 
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

  }

 
  @Override
  public void autonomousInit() {

   // startTime = Timer.getFPGATimestamp(); 
    m_autonomousCommand = getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  
 
  @Override
  public void autonomousPeriodic() {

    /*
    double time = Timer.getFPGATimestamp();

    if (time - startTime < 3) {

      outtake.set(.125);;
      outtake2.set(.5);

      uptake.set(1);

    } else if (time - startTime > 3 && time - startTime < 8) {

        leftSpeedGroup.set(.2);
        rightSpeedGroup.set(-.2);

      } else {

        robotDrive.arcadeDrive(0, 0);

        outtake2.set(0);
        outtake.set(0);
        uptake.set(0);

    }
  }

*/
  }
  
 
  @Override
  public void teleopInit() {
 
   
  }
 
  @Override
  public void teleopPeriodic() {
 
    
      compressor.enabled();
 

// start of controller one code

    if(controller.getTriangleButton()) {
      shifter.set(true);
    } else if(controller.getSquareButton()){
      shifter.set(false);
    }

    if(controller.getR2ButtonPressed()){

      dropIntake1.set(Value.kForward);
      dropIntake2.set(Value.kForward);

      intakeGroup.set(.5);
     
    }else if(controller.getR2ButtonReleased()) {

      dropIntake1.set(Value.kReverse);
      dropIntake2.set(Value.kReverse);
 
      intakeGroup.set(0);

    }

// end of controller one code 


// start of controller two code

    if(controller2.getCircleButtonPressed()) { // upper hub right on outside of tarmac
      outtake2.set(1);
      outtake.set(1);
     
    }else if(controller2.getCircleButtonReleased()) {
      outtakeGroup.set(0);
    }


    if(controller2.getCrossButtonPressed()){ // try tmr good for upper hub
      outtake2.set(.25);
      outtake.set(1);
     
    }else if(controller2.getCrossButtonReleased()) {
      outtakeGroup.set(0);
    } 

    if(controller2.getTriangleButtonPressed()){ // low hub right next to fender or two feet away
      outtake2.set(.5);
      outtake.set(.125);
     
    }else if(controller2.getTriangleButtonReleased()) {
      outtakeGroup.set(0);
    }

 
    if(controller2.getR2Button()){ // uptake with middle wheel
      
      uptake.set(1);
      intakeMotor2.set(.5);
     
    }else if(controller2.getL2Button()) { // reverse uptake with middle wheel

      uptake.set(-1);
      intakeMotor2.set(-.5);

    } else {

      uptake.set(0);
      intakeMotor2.set(0);
      
    }

    if(controller2.getR1Button()){ // middle wheel

      intakeMotor2.set(.5);
     
    }else if(controller2.getL1Button()) { // reverse  middle wheel

      intakeMotor2.set(-.5);

    } else {

      intakeMotor2.set(0);
      
    }
 
    robotDrive.arcadeDrive(controller.getRawAxis(2), -controller.getRawAxis(1));
    
  }

 
  @Override
  public void disabledInit() {}
 
  @Override
  public void disabledPeriodic() {}
 
  @Override
  public void testInit() {}
 
  @Override
  public void testPeriodic() {
 
   
 
  }
}
 