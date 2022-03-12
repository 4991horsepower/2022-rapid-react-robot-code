// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RamseteController ramsete = new RamseteController();

  private XboxController m_driverController = new XboxController(0);
  private XboxController m_copilotContoller = new XboxController(1);
  
  private CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushless);  
  private CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax m_rearRight = new CANSparkMax(4, MotorType.kBrushless);
  private RelativeEncoder m_leftEnc;
  private RelativeEncoder m_rightEnc;
  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;

  // shoopter
  private CANSparkMax m_shooter = new CANSparkMax(10, MotorType.kBrushless);
  private SparkMaxPIDController m_shooterPIDController;

  // climbler
  private CANSparkMax m_climber = new CANSparkMax(8, MotorType.kBrushless);
  private RelativeEncoder m_climber_enc;

  // int ache
  private CANSparkMax m_intake = new CANSparkMax(5, MotorType.kBrushless);

  // soul annoyed (intake)
  private Solenoid intake1 = new Solenoid(PneumaticsModuleType.REVPH, 0);
 
  //kicker
  private Solenoid kicker = new Solenoid(PneumaticsModuleType.REVPH, 5);

  // Climber
  private Solenoid fingy1 = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private Solenoid fingy2 = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private Solenoid fingy3 = new Solenoid(PneumaticsModuleType.REVPH, 3);
  private Solenoid fingy4 = new Solenoid(PneumaticsModuleType.REVPH, 4);
  private int stage = 0; // needs to be defined in a scope outside teleop

  // sensor!!!!
  private DigitalInput fwdLimitSwitch = new DigitalInput(0);
  private DigitalInput revLimitSwitch = new DigitalInput(1);

  //lifter
  private CANSparkMax m_lifter = new CANSparkMax(7, MotorType.kBrushless);

  // belt drive
  private CANSparkMax m_belt = new CANSparkMax(6, MotorType.kBrushless);

  //comp ressor
  private Compressor compressorator = new Compressor(PneumaticsModuleType.REVPH);
  
  //color boi
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorBoi = new ColorSensorV3(i2cPort);  
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429); //values subject to change
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

  private String teamColor;
  private String colorString;

  private Gyro m_gyro = new ADXRS450_Gyro();

  Trajectory trajectory = new Trajectory();

  // timer
  private Timer auto_timer = new Timer();
  private Timer climb_timer = new Timer();

  //analog
  private AnalogInput ball_detect = new AnalogInput(0);

  // network table
  private NetworkTableEntry isRedAlliance;

  // Button States
  private boolean intakeIn;
  private boolean intakeToggle_prev;
  private boolean shooterToggle_prev;
  private boolean shooterOn;
  private boolean spinning;

  private DifferentialDriveOdometry m_odometry  = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.555625);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    NetworkTableInstance NetTableInst = NetworkTableInstance.getDefault();
    NetworkTable table = NetTableInst.getTable("FMSInfo");
    isRedAlliance = table.getEntry("isRedAlliance");
    if(isRedAlliance.getBoolean(true))
    {
      teamColor = new String("red");
    }
    else{
      teamColor = new String("blue");
    }

    String[] auto_modes = {"Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5"};
    SmartDashboard.putStringArray("Auto List", auto_modes);
    
    m_frontLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();

    m_rearLeft.setInverted(false);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(false);
    m_frontRight.setInverted(true);
  
    m_rearLeft.follow(m_frontLeft);
    m_frontRight.follow(m_rearRight);

    m_leftEnc = m_frontLeft.getEncoder();
    m_leftEnc.setPositionConversionFactor(0.0585);
    m_leftEnc.setPosition(0.0);

    m_rightEnc = m_rearRight.getEncoder();
    m_rightEnc.setPositionConversionFactor(0.0585);
    m_rightEnc.setPosition(0.0);

    m_leftPIDController = m_frontLeft.getPIDController();
    m_leftPIDController.setFF(0.00018);
    m_leftPIDController.setP(0.00025);
    m_leftPIDController.setI(0.0);
    m_leftPIDController.setD(0.0);

    m_rightPIDController = m_rearRight.getPIDController();
    m_rightPIDController.setFF(0.00018);
    m_rightPIDController.setP(0.00025);
    m_rightPIDController.setI(0.0);
    m_rightPIDController.setD(0.0);

    m_climber.restoreFactoryDefaults();
    m_climber_enc = m_climber.getEncoder();

    // Shooter
    m_shooter.restoreFactoryDefaults();
    m_shooter.setInverted(true);
    m_shooterPIDController = m_shooter.getPIDController();
    m_shooterPIDController.setFF(0.00025);
    m_shooterPIDController.setP(0.0008);
    m_shooterPIDController.setI(0.0);
    m_shooterPIDController.setD(0.0);
    m_shooterPIDController.setOutputRange(-0.1, 1);
    
    //colorator
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.setConfidenceThreshold(.2);

    compressorator.enableAnalog(100, 120);

    intakeIn = true;
    spinning = false;

    m_climber_enc.setPosition(0);
    m_climber_enc.setPositionConversionFactor(1.0/375.0);

    shooterOn = false;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Color detectedColor = colorBoi.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kRedTarget){
      colorString = "red";
    }
    else if (match.color == kBlueTarget){
      colorString = "blue";
    }
    else {
      colorString = "unknown";
    }
    m_odometry.update(m_gyro.getRotation2d(), m_leftEnc.getPosition(), m_rightEnc.getPosition());
    //System.out.println(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  @Override
  public void autonomousInit() {
    String m_autoSelected = SmartDashboard.getString("Auto Selector", "Auto 1");
    System.out.println("Auto selected: " + m_autoSelected);
    String trajectoryJSON;
    if(m_autoSelected.equals("Test"))
    {
      trajectoryJSON = "paths/output/getBall.wpilib.json";
    }
    else {
      trajectoryJSON = "paths/output/getBall.wpilib.json";
    }

    // Moved here so we can choose a filename based on auto selected
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    m_rightEnc.setPosition(0.0);
    m_leftEnc.setPosition(0.0);
    m_odometry.resetPosition(trajectory.getInitialPose(), m_gyro.getRotation2d());

    auto_timer.reset();
    auto_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    Trajectory.State goal = trajectory.sample(auto_timer.get());
    //System.out.print("Goal: ");
    //System.out.println(goal);
    ChassisSpeeds adjustedSpeeds = ramsete.calculate(m_odometry.getPoseMeters(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond * 60 / 0.0585;
    double right = wheelSpeeds.rightMetersPerSecond * 60 / 0.0585;

    //m_frontLeft.set(left);
    //m_rearRight.set(right);

    m_leftPIDController.setReference(left, CANSparkMax.ControlType.kVelocity);
    m_rightPIDController.setReference(right, CANSparkMax.ControlType.kVelocity);

    /*
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here 
        
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }    
    */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    stage = 0;
    climb_timer.start();
    m_climber_enc.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    //System.out.println(teamColor);

    // lame drivetrain stuff i suppose - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    double reverse = m_driverController.getLeftTriggerAxis();
    double forward = m_driverController.getRightTriggerAxis();
    double front_back = reverse < 0.01 ? forward : -reverse;
    double turnVal = -m_driverController.getLeftX();

    front_back = front_back > 0 ? Math.pow(Math.abs(front_back), 3) : -Math.pow(Math.abs(front_back), 3);
    turnVal = turnVal > 0 ? 0.5*Math.pow(Math.abs(turnVal), 3) : -0.5*Math.pow(Math.abs(turnVal), 3);

    double left = 0, right = 0;
    int backupType = 0;
   
    // code to switch between regular reversing (like a car; 0) and mirrored reversing (1)
    // case 2 to disable drivetrain (for testing safety purposes)
    switch(backupType){
      case 0:
        if(front_back >= 0){ // when the robot is going forwards or not moving
          right = front_back + turnVal;
          left = front_back - turnVal;
        }
        else if(front_back < 0){ // when the robot is reversing
          right = front_back - turnVal;
          left = front_back + turnVal;
        }
        break;
      case 1:
          right = front_back + turnVal;
          left = front_back - turnVal;
        break;
      case 2:
        right = 0;
        left = 0;
        break;
    }

    m_frontLeft.set(left);
    m_rearRight.set(right);
    
    // end of the lame drivetrain stuff i suppose ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    

    // int ache code - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    
    boolean intakeToggle = m_driverController.getLeftBumper();
    boolean motorOn = m_driverController.getAButton();

    // toggle intake in/out (solenoids)
   if(intakeToggle == true && intakeToggle_prev == false){
    intakeIn = !intakeIn;
   }

   intakeToggle_prev = intakeToggle;
   intake1.set(intakeToggle);

    // only let motor spin if intake is out
    if(/*!intakeIn && */motorOn){
      m_intake.set(-.5); // edit for correct rotation direction (linear relationship)
    }
    else {
      m_intake.set(0);
    }
    
    // end int ache code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



    // belt code - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    boolean beltForward = m_copilotContoller.getYButton();
    boolean beltBack = m_copilotContoller.getBButton();
    if(beltForward && !beltBack){
      m_belt.set(.6);
    }
    else if(!beltForward && beltBack){
      m_belt.set(-.6);
    }
    else{
      //m_belt.set(0);
    }
   // System.out.println(colorString + ", " + colorBoi.getProximity());
    if (!intakeIn){
      if (ball_detect.getAverageValue() <= 500){
        if(colorBoi.getProximity() > 200){
          if(colorString.equals(teamColor)){
            m_belt.set(-0.6);
            m_lifter.set(-0.45);
            kicker.set(true);
          }
          else{
            m_belt.set(-0.6);
            m_lifter.set(0);
            kicker.set(false);
          }
        }
        else {
          m_belt.set(-0.6);
        }
      }
    else{
      if(colorBoi.getProximity() > 200){
        if(colorString.equals(teamColor)){
          m_belt.set(0);
          m_lifter.set(0);
          kicker.set(false);
        }
        else{
          m_belt.set(-0.6); 
          m_lifter.set(0);
          kicker.set(true);
        }
      }
      else {
        m_belt.set(-0.6);
      }
    }
  }

    // end belt code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    

    // shoopter code - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
             
    // to use for testing motor speed (dpad)
    // POV: 0 up, 90 right, 180 down, 270 left
/*    double motorSpeed = 0;

    if(m_copilotContoller.getPOV() == 0){
      motorSpeed += 0.01;
    }
    else if(m_copilotContoller.getPOV() == 180){
      motorSpeed -= 0.01;
    }
    if(motorSpeed < -1){
      motorSpeed = -1;
    }
    else if (motorSpeed > 1){
      motorSpeed = 1;
    }
*/
   
    
    /*
    if(!spinning){
      if(toggleSpin){
        m_shooter.set(-0.35);
        spinning = true;
      }
      else{
        m_shooter.set(0);
        spinning = false;
      }
    }*/
     // toggle intake in/out (solenoids)
   boolean toggleSpin = m_copilotContoller.getRightBumper();
   if(toggleSpin == true && shooterToggle_prev == false){
    shooterOn = !shooterOn;
   }

   shooterToggle_prev = toggleSpin;
   if (shooterOn){
     m_shooterPIDController.setReference(1000, CANSparkMax.ControlType.kVelocity);
     spinning = true;
    }
    else{
      m_shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
      spinning = false;
    }

    // l i f t    b a l l 
    boolean shoopt = m_copilotContoller.getLeftBumper();
  //  if(spinning){
      if(shoopt){
        m_lifter.set(.45);
      }
   // System.out.println(ball_detect.getAverageValue());
  //  }   
    // end shoopter code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   
    // fingy test code //////////////////////////////////////////////////////////////////////////////////////////


    if(m_copilotContoller.getPOV() == 0){
      fingy1.set(true);
      fingy2.set(false);
      fingy3.set(false);
      fingy4.set(false);
    }
    else if(m_copilotContoller.getPOV() == 90){
      fingy1.set(false);
      fingy2.set(true);
      fingy3.set(false);
      fingy4.set(false);
    }
    else if(m_copilotContoller.getPOV() == 180){
      fingy1.set(false);
      fingy2.set(false);
      fingy3.set(true);
      fingy4.set(false);
    }
    else if(m_copilotContoller.getPOV() == 270){
      fingy1.set(false);
      fingy2.set(false);
      fingy3.set(false);
      fingy4.set(true);
    }

    // end fingy test code ///////////////////////////////////////////////////////////////////////////////////


    // climbler code - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


    // climber code by ethan            
    // finger setup: 1 and 2 on one side; when the robot faces right, 1 and 2 start on the right
    // false is open and true is closed
    /*

      visual description (robot facing right):

     fingy3 -------------------------------------------------------- fingy 2
          |                         |_|                             |
     fingy4 -------------------------------------------------------- fingy 1

    */

    // set fingers to one side open, rotate right side up to hit bar (so that only fingy1 hits the bar)
    if(m_copilotContoller.getStartButton()){
      switch(stage){
        case 0:
            // Rotate climber to vertical (-0.25 revolutions)
          m_climber.set(0.25);
          if(m_climber_enc.getPosition() >= 0.25)
          {
            stage = 1;
          }
          break;
        case 1:
          // Climber is in position, close fingy 2
          m_climber.set(0);
          fingy1.set(true);
          fingy2.set(false);
          fingy3.set(true);
          fingy4.set(false);
          if(revLimitSwitch.get() == false)
          {
            climb_timer.reset();
            stage = 2;
          }
          break;
        case 2:
          // Give it a second to fully close fingy 3
          m_climber.set(0);
          fingy1.set(false);
          fingy2.set(false);
          fingy3.set(true);
          fingy4.set(false);   
          if(climb_timer.get() > 1.0)
          {
            stage = 3;
          } 
          break;
        case 3:
          // Rotate until in the next position, close fingy 3 for stopping point on rotation
          m_climber.set(-1.0);
          fingy1.set(false);
          fingy2.set(false);
          fingy3.set(true);
          fingy4.set(false);

          if(fwdLimitSwitch.get() == false)
          {
            climb_timer.reset();
            stage = 4;
          }
          break;
        case 4:
          // Close fingy 1
          m_climber.set(0);
          fingy1.set(false);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(false);
          if(climb_timer.get() > 1.0)
          {
            climb_timer.reset();
            stage = 5;
          }
          break;
        case 5:
          // Release fingy 2 and 3
          m_climber.set(0);
          fingy1.set(true);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(true);
          if(climb_timer.get() > 1.0)
          {
            climb_timer.reset();
            stage = 6;
          }
          break;
        case 6:
          // Bounce the robot to unlock the finger
          m_climber.set(1);
          fingy1.set(true);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(true);       
          if(climb_timer.get() > 0.25)
          {
            climb_timer.reset();
            stage = 7;
          }
          break;
        case 7:
          // Bounce the robot to unlock the finger
          m_climber.set(-1);
          fingy1.set(true);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(true);       
          if(climb_timer.get() > 0.25)
          {
            climb_timer.reset();
            stage = 8;
          }
          break;
        case 8:
          // Climber is in position, close fingy 2
          m_climber.set(-1.0);
          fingy1.set(true);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(false);
          if(revLimitSwitch.get() == false)
          {
            climb_timer.reset();
            stage = 9;
          }
          break;
        case 9:
        // Close all fingers
          m_climber.set(0);
          fingy1.set(false);
          fingy2.set(false);
          fingy3.set(false);
          fingy4.set(false);
          if(climb_timer.get() > 1.0)
          {
            climb_timer.reset();
            stage = 10;
          }
          break;
        case 10:
          // Release fingers 2 and 3
            m_climber.set(0);
            fingy1.set(false);
            fingy2.set(true);
            fingy3.set(true);
            fingy4.set(false);
            if(climb_timer.get() > 1.0)
            {
              climb_timer.reset();
              stage = 11;
            }
            break;
        case 11:
            // Bounce the robot to unlock the finger
            m_climber.set(1);
            fingy1.set(true);
            fingy2.set(false);
            fingy3.set(false);
            fingy4.set(true);       
            if(climb_timer.get() > 0.25)
            {
              climb_timer.reset();
              stage = 12;
            }
            break;
          case 12:
            // Bounce the robot to unlock the finger
            m_climber.set(-1);
            fingy1.set(true);
            fingy2.set(false);
            fingy3.set(false);
            fingy4.set(true);       
            if(climb_timer.get() > 0.25)
            {
              climb_timer.reset();
              stage = 3;
            }
            break;
      }
    }
    else{
      m_climber.set(0);
    }
    // end climbler code ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ^ since this comment is related to the climber, please don't put it outside the teleop call
  } 



  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    double manual_climber = m_copilotContoller.getLeftY();
    if(Math.abs(manual_climber) < 0.1){
      manual_climber = 0;
    }
    m_climber.set(manual_climber);
  
  if(m_copilotContoller.getPOV() == 0){
    fingy1.set(true);
    fingy2.set(false);
    fingy3.set(false);
    fingy4.set(false);
  }
  else if(m_copilotContoller.getPOV() == 90){
    fingy1.set(false);
    fingy2.set(true);
    fingy3.set(false);
    fingy4.set(false);
  }
  else if(m_copilotContoller.getPOV() == 180){
    fingy1.set(false);
    fingy2.set(false);
    fingy3.set(true);
    fingy4.set(false);
  }
  else if(m_copilotContoller.getPOV() == 270){
    fingy1.set(false);
    fingy2.set(false);
    fingy3.set(false);
    fingy4.set(true);
  }
  else
  {
    fingy1.set(false);
    fingy2.set(false);
    fingy3.set(false);
    fingy4.set(false);
  }
}
}
