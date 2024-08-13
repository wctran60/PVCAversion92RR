package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;


public class RobotHardware {
    // Declare OpMode members.  Important to assign a starting value to the variable, even if it is null.

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor armMotor = null;







    private Servo   leftHand = null;
    private Servo  rightHand = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED = 0.02 ;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;


    //IMU sensor object.
    private IMU imu;



    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

        /**Initialize all the robot's hardware.  This method must be called ONCE when the OpMode is initialized.
         * All of the hardware devices are accessed via the hardware map, and initialized.
         */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "armMotor");



        /**  IMPORTANT Drive Information. Test your motor directions.            !!!!!
         *  Most robots need the motors on one side to be reversed to drive forward.
         *  The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
         *  Start out with the reversals here, BUT when you first test your robot, push
         *        the left joystick forward and observe the direction the wheels turn.
         *  Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
         *  Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        */




        // TODO:  If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Define and initialize ALL other installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);


/**FOR MECANUM Wheel driving
        //IMU setup to use for field centric robot driving.
        //Define and initialize IMU sensor on new Control Hub--new orientation of the control hub
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        //Define how the hub is mounted on robot to get correct Yaw, Pitch and Roll values.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //Initialize IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
*/
        //IMU setup to use for field centric robot driving.
        //Define and initialize IMU sensor on new Control Hub--new orientation of the control hub








        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }




    /**  FOR TANK DRIVE--CALCULATE POWER and PASS IT TO EACH MOTOR
     * Calculates the left/right motor powers required to achieve the requested robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    public void driveTankRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
    double left = Drive+Turn;
    double right = Drive+Turn;
        // Scale the values so neither exceed +/- 1.0
        double max  = Math.max (Math.abs(left),Math.abs(right));
        if(max >1.0) {
            left /= max;
            right /= max;
        }
        // send power values to both motors.  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
        frontRightMotor.setPower(right);
        frontLeftMotor.setPower(left);
    }


    /** Pass requested ARM power to the appropriate hardware drive motor
     * @param power driving power (-1.0 to 1.0)
    */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }




    /** Send two hand-servos to opposing (mirrored) positions, based on the passed offset.
     * @param offset
    */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset,-0.5,0.5);
        leftHand.setPosition(MID_SERVO+offset);
        rightHand.setPosition(MID_SERVO-offset);

    }

}










