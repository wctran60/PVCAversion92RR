package org.firstinspires.ftc.teamcode.PVCAanswer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** LinearOpMode MODE TEMPLATE
 runOpMode(): Code inside this method will run exactly once after you press the INIT button.
        This is where you should put all code for the OpMode.
 waitForStart(): This method pauses the Op-Mode until you press the START button on the driver station.
  isStarted(): returns true if the START button has been pressed, otherwise it returns false.
  isStopRequested(): returns true if the STOP button has been pressed, otherwise it returns false.
  idle(): calls Thread.yield, allowing other threads at the same priority level to run.
 opModeIsActive(): returns isStarted() && !isStopRequested() and calls idle().
    example:  while (!isStarted() && !isStopRequested()) {    }: loop between INIT and START for 30 second AUTONOMOUS program
    example: while (opModeIsActive()) {   }: loop after START to run robot during 2-minute TELEOP portion
 opModeInInit(): returns !isStarted() && !isStopRequested() and does not call idle().
 */


@TeleOp(name="Mecanum Drive: LinearOpMode", group="LinearOpMode")
//for 30-second Autonomous, use:  @Autonomous(name = "Auto example", group="Blue")

public class TeleOp_Linear_Mecanum extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot...." to access this class.
    RobotHardware robot       = new RobotHardware(this);

    //Driving orientation:  default "fieldOriented" (driver's perspective driving), but can be "robotOriented"
    String drivingOrientation = "fieldOriented";

    //declare variables for drive method
    double x, y, rx;
    double botHeadingRadian, botHeadingDegree;
    double power, motorPowerDefault, powerChange;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
    //this method will run exactly once after you press the INIT button.

        // initialize all the hardware, using the separated hardware class
        robot.init();

        double arm          = 0;
        double handOffset   = 0;

//*****LOOP wait for start, INIT LOOP**************************************************************
        while (!isStarted() && !isStopRequested()) {

        }

//*****START LOOP**********************************************************************************
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //***DRIVING ORIENTATION
            if (gamepad1.left_stick_button) {drivingOrientation = "robotOriented";}
            if (gamepad1.right_stick_button) {drivingOrientation = "fieldOriented";}

            //TODO: Sign for gamepad input need to be tested confirm sign to be +positive or -negative.
            y = -gamepad1.left_stick_y;           // TODO: Remember,joystick value is reversed!
            x = gamepad1.left_stick_x * 1.1;      // TODO: Adjust factor--Counteract imperfect strafing factor, default=1.1
            rx = gamepad1.right_stick_x;

            //Cancel angle movement of gamepad left stick, i.e., make move either up/down or right/left
            if (Math.abs(y) >= Math.abs(x)) {
                y = y;
                x = 0;
            } else {
                y = 0;
                x = x;
            }

            //SPEED ADJUSTMENT--default, speed up, slow down
            //SLOW DOWN = RIGHT LOWER TRIGGER (lower of the top side button).  SPEED UP = RIGHT UPPER BUMPER
            motorPowerDefault = 0.5; //robot motor speed at 50% max
            if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.left_trigger > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.left_trigger > 0.1) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.left_trigger > 0.1)) {
                powerChange = -0.3;
            } else if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_trigger > 0.1)) {
                powerChange = 0.3;
            } else {
                powerChange = 0;
            }
            power = motorPowerDefault + powerChange;

            //robot heading is based on Gyro sensor on IMU in the CONTROL HUB
            botHeadingRadian = robot.getYawRadian();
            botHeadingDegree = Math.toDegrees(botHeadingRadian);

            robot.driveMecanum(y, x, rx, power, botHeadingRadian, drivingOrientation);


            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.right_bumper)
                handOffset += robot.HAND_SPEED;
            else if (gamepad1.left_bumper)
            handOffset -= robot.HAND_SPEED;
            handOffset = Range.clip(handOffset, -0.5, 0.5);
            // Move both servos to new position.  Use RobotHardware class
            robot.setHandPositions(handOffset);


            // Use gamepad buttons to move arm up (Y) and down (A).  Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y)
                arm = robot.ARM_UP_POWER;
            else if (gamepad1.a)
                arm = robot.ARM_DOWN_POWER;
            else
                arm = 0;
            robot.setArmPower(arm);


            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Present Robot Driving Orientation = ", drivingOrientation);
            telemetry.addData("Field-Oriented/Robot-Oriented", "Left button/Right button");
            telemetry.addData("-", "-------");
            telemetry.addData("Drive Forward/Backward", "LEFT Stick Up/Down");
            telemetry.addData("Strafe Left/Right", "LEFT Stick Left/Right");
            telemetry.addData("Turn Left/Right", "RIGHT Stick Left/Right");
            telemetry.addData("-", "-------");
            telemetry.addData("Arm Up/Down", "Y & A Buttons");
            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
            telemetry.addData("-", "-------");

            telemetry.addData("Robot Drive/Turn Power", "%.2f", power);
            telemetry.addData("Robot Slow/Fast","Left Trigger/Right Trigger");
            telemetry.addData("-", "-------");

            telemetry.addData("Arm Power",  "%.2f", arm);
            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.addData("-", "-------");

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
