/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.peripheral.INorFlashSector;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.camera.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team16352012 extends SimpleRobot {

//commented out so we can use the Jaguars for the tower
    private RobotDrive driveTrain = new RobotDrive(1, 2);
    //private Joystick leftStick = new Joystick(1);
    private GameController controller = new SingleGameController(1);
    //private Joystick rightStick = new Joystick(2);
    //private Joystick extraStick = new Joystick(3);
    //private Compressor robotComp = new Compressor(2, 2);
    private Victor roller = new Victor(5); //belt feeder at the bottom
    private Victor conveyorBelt = new Victor(6);
    private Victor shooter = new Victor(7);
    private AxisCamera camera = AxisCamera.getInstance();
    private DigitalInput digitalSensor = new DigitalInput(1);
    private DigitalInput Switch = new DigitalInput(2);
    private DigitalInput topSensor = new DigitalInput(14);
    private DigitalInput botSensor = new DigitalInput(13);
    private StopWatch beltStopDelay;
    private StopWatch shooterStartDelay;
    private StopWatch shooterStopDelay;
    private double beltMaxSpeed = 0.8;
    private double basketSpeed = 0.0;
    private double period = 0.0;
    private double rpm = 0.0;
//    private DriverStation driverStation;
    private KinectStick leftArm = new KinectStick(1);
    private KinectStick rightArm = new KinectStick(2);
    private Relay shift = new Relay(1);
    private Counter rotationCounter;

    public Team16352012() {
        camera.writeResolution(AxisCamera.ResolutionT.k640x480);
        //camera.writeResolution(AxisCamera.ResolutionT.k320x240);
        //camera.writeResolution(AxisCamera.ResolutionT.k160x120);
        //camera.writeMaxFPS(32);
        getWatchdog().setExpiration(.5);


        //start compressor and fill tanks
        //robotComp.start();
        beltStopDelay = new StopWatch(500);
        shooterStartDelay = new StopWatch(1800); //to be tested, will not be needed when we have RPM measure.
        shooterStopDelay = new StopWatch(2000);
        rotationCounter = new GearTooth(digitalSensor);
        //Set claw to close direction
        shift.setDirection(Relay.Direction.kForward);
    }

    public void robotInit() {
//        driverStation = DriverStation.getInstance();
    }

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        boolean basketDetermine;//true = higher basket; false = lower basket

        boolean Ashooting = false;
        boolean AwasShooting = false;
        boolean AwasBallAtTop = false;
        boolean AballAtTop = false;
        int count = 0;

        while (isEnabled()) {
            /**
             * KinectStick axis values are accessed identically to those of a
             * joystick In this example the axis values have been scaled by ~1/3
             * for safer operation when learning to use the kinect.
             */
            driveTrain.tankDrive(leftArm.getY() * .75, rightArm.getY() * .75);

//            basketDetermine = Switch.get();

//            if (rightStick.getRawButton(7)) {
            if (controller.getButtonY()) {
                basketSpeed = 0.75;
//                System.out.println("On");
            } else {
                basketSpeed = 0.40;
//                System.out.println("Off");
            }


            AballAtTop = !(topSensor.get());

            if (leftArm.getRawButton(7) && rightArm.getRawButton(7)) {
                Timer.delay(0.3);
                Ashooting = true;
                System.out.println("shooter pressed");
            }

            if ((!AwasShooting) && (Ashooting)) {
                shooterStartDelay.start();
                System.out.println("starter starts");
            }

            //shooter spinning decision
            if (Ashooting) {
                //determine current shooter state
                //TODO
                shooter.set(basketSpeed);

                if ((AwasBallAtTop) && (!AballAtTop)) {
                    count = count + 1;
                }

                if (shooterStartDelay.delayExpired()) {
                    conveyorBelt.set(beltMaxSpeed);
                } else {
                    conveyorBelt.set(0.0);
                }

                AwasBallAtTop = AballAtTop;

                if (count == 1) {
                    conveyorBelt.set(0.0);
                    Ashooting = false;
                    count = 0;
                    System.out.println("shooting = false");
                    AwasBallAtTop = false;
                }


            } else {

                shooter.set(0.0);

            }

            AwasShooting = Ashooting;
            Timer.delay(0.01);
        }

    }

    public void operatorControl() {
        boolean isTankDrive = true; //determine drive type
        boolean DriveSpeed = true; //determine drive speed

        boolean rollFeeder = false;  //is the roll at the bottom of the belt rolling
        boolean ballAtBottom = false;  //is there a ball detected at the bottom
        boolean wasBallAtBottom = false; //was the ball detected at the bottom in the last loop
        boolean ballAtTop = false;  //is there a ball detected at the top
        boolean wasBallAtTop = false;

        boolean shooting = false;
        boolean startingShooterMode = false;
        boolean wasShooting = false;
        boolean stopBelt = false;

        boolean clearBotDelayMode = false;  //are we in delay mode (keep moving the ball for a while after the ball cleared the bottom sensor
        boolean stopShooterMode = false;

        boolean shooterHeatUp = false;
        boolean warmUpMode = true;

        //start compressor and fill tanks
        //robotComp.start();
        System.out.println("CP starts");


        getWatchdog().setEnabled(true);

        while (isOperatorControl() && isEnabled()) {
            getWatchdog().feed();

            //drive type decison
            if (controller.getButtonX()) {
                Timer.delay(0.5); //there has to be a better way to read reliably
                isTankDrive = !(isTankDrive);
            }

            //Set the driving speed (currently not used)
            if (controller.getButtonA()) {
                Timer.delay(0.5); //there has to be a better way to read reliably
                DriveSpeed = !(DriveSpeed);
            }
//            * This will control the claw by pressing buttons 4 and 5
//             * on the extra joystick.
//             */
//            if (leftStick.getRawButton(4)) {
//                shift.setDirection(Relay.Direction.kForward);
//                shift.set(Relay.Value.kOn);
//                System.out.println("button4/n");
//            } else if (leftStick.getRawButton(5)) {
//                shift.setDirection(Relay.Direction.kReverse);
//                shift.set(Relay.Value.kOn);
//                System.out.println("button5/n");
//            } else {
//                shift.set(Relay.Value.kOff);
//            }

            //commented out so we can use the Jaguars to drive the tower. 
            //drive train decision
            if (isTankDrive) {
                if (DriveSpeed) {
                    driveTrain.tankDrive(controller.getRightJoystickUpDown(), controller.getLeftJoystickUpDown());
                } else {
                    driveTrain.tankDrive(controller.getRightJoystickUpDown() * .75, controller.getLeftJoystickUpDown() * .75);
                }
            } else {
                if (DriveSpeed) {
                    driveTrain.arcadeDrive(controller.getLeftJoystickUpDown(), controller.getLeftJoystickLeftRight(), true);
                } else {
                    driveTrain.arcadeDrive(controller.getLeftJoystickUpDown() * .75, controller.getLeftJoystickLeftRight() * .75, true);
                }
            }

//            if (controller.getButtonB()) {
//                shooterHeatUp = !shooterHeatUp;
//            }

            if (controller.getButtonB()) {
                basketSpeed = 0.0;
                System.out.println("B is pressed");
            }
//
//            if (extraStick.getRawButton(5)) {
//                basketSpeed = 0.5;
//            }
//
//            if (extraStick.getRawButton(6)) {
//                basketSpeed = 0.6;
//            }
//
//            if (extraStick.getRawButton(7)) {
//                basketSpeed = 0.7;
//            }
//
//            if (extraStick.getRawButton(8)) {
//                basketSpeed = 0.8;
//            }
//
//            if (extraStick.getRawButton(9)) {
//                basketSpeed = 0.9;
//            }
//
//            if (extraStick.getRawButton(10)) {
//                basketSpeed = 1.0;
//            }

            ballAtBottom = !(botSensor.get());
            ballAtTop = !(topSensor.get());

            //read button that controls the bottom feed roller.
            if (controller.getFrontLeftButton()) {
                Timer.delay(0.3);
                rollFeeder = !rollFeeder;
            }

            //bottom feed roll decision
            if (rollFeeder) {
                roller.set(-1.0);
            } else {
                roller.set(0.0);
            }

            //read button that controls the spinning shooter
            if (controller.getFrontRightButton()) {
                Timer.delay(0.3);
                shooting = true;
                System.out.println("shooter pressed");
            }

            if ((!wasShooting) && (shooting)) {
                shooterStartDelay.start();
                System.out.println("starter starts");
            }

            //shooter spinning decision
            if (shooting) {
                //determine current shooter state
                //TODO
//                shooter.set(driverStation.getAnalogIn(1) / 5);
                shooter.set(basketSpeed);

                if ((wasBallAtTop) && (!ballAtTop)) {
                    stopBelt = true;
                    System.out.println("Belt stop");
                }

                if (shooterStartDelay.delayExpired()) {
                    conveyorBelt.set(beltMaxSpeed);
                } else {
                    conveyorBelt.set(0.0);
                }

                wasBallAtTop = ballAtTop;

                if (stopBelt) {
                    conveyorBelt.set(0.0);
                    shooting = false;
                    stopBelt = false;
                    System.out.println("shooting = false");
                    stopShooterMode = true;
                    wasBallAtTop = false;
                    warmUpMode = false;
                }


            } else {

                if (shooterStopDelay.delayExpired() && (stopShooterMode)) {
                    shooter.set(0.0);
                    warmUpMode = true;
                    shooterHeatUp = false;
                } else if (stopShooterMode = true) {
//                    shooter.set(driverStation.getAnalogIn(1) / 5);
                    shooter.set(basketSpeed);
                } else {
                    shooter.set(0.0);
                }

                if ((shooterHeatUp) && (warmUpMode)) {
                    shooter.set(1.0);
                } else {
                    shooter.set(0.0);
                }
                //read light sensors to determine belt state

                //if in the previous loop ball was blocking bottom sensor
                //  and in this loop ball is not blocking the sensor we need
                //  to start the asynchronous timer (stopWatch) and enter bottom
                //  clearing delay mode
                if ((wasBallAtBottom) && (!ballAtBottom)) {
                    clearBotDelayMode = true;
                    beltStopDelay.start();
                }

                //TODO: stop the roller or spin it the other way in
                //clearing bottom mode

                //belt driving decision determined by state
                if ((clearBotDelayMode) && (!ballAtTop)) {
                    //clearing bottom mode
                    if (beltStopDelay.delayExpired()) {
                        conveyorBelt.set(0.0);
                        clearBotDelayMode = false;
                    } else {
                        conveyorBelt.set(beltMaxSpeed);
                    }
                } else if ((ballAtBottom) && (!ballAtTop)) {
                    conveyorBelt.set(beltMaxSpeed);
                } else {
                    conveyorBelt.set(0.0);
                }
            }

            if ((wasShooting) && (!shooting)) {
                shooterStopDelay.start();
                System.out.println("stopper starts");
            }

            wasShooting = shooting;
            wasBallAtBottom = ballAtBottom;
            
            period = rotationCounter.getPeriod();
            rpm = 60 / period;
            
        }
    }

    protected void disabled() {
        System.out.println("Robot has been disabled");
    }
}
