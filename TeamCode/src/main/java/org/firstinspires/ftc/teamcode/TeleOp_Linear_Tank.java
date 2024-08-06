package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


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


@TeleOp(name="Tank Drive: LinearOpMode TEST", group="LinearOpMode")
//for 30-second Autonomous, use:  @Autonomous(name = "Auto example", group="Blue")

public class TeleOp_Linear_Tank extends LinearOpMode {
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
    //this method will run exactly once after you press the INIT button.






        // initialize all the hardware, using the separated hardware class
        robot.init();

//*****LOOP wait for start, INIT LOOP**************************************************************
        while (!isStarted() && !isStopRequested()) {

        }

//*****START LOOP**********************************************************************************
        // Wait for the game to start (driver presses PLAY--START)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot forward and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            //drive = ...;
            //turn  =  ...;

            // Combine drive and turn for blended motion. Use RobotHardware class
            //robot.driveTankRobot(drive, turn);

            // Use gamepad left & right Bumpers to open and close the claw
            // Use the SERVO constants defined in RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range



            // Move both servos to new position.  Use RobotHardware class



            // Use gamepad buttons to move arm up (Y) and down (A).  Use the MOTOR constants defined in RobotHardware class.




            // Send telemetry messages to explain controls and show robot status
//            telemetry.addData("Drive", "Left Stick");
//            telemetry.addData("Turn", "Right Stick");
//            telemetry.addData("Arm Up/Down", "Y & A Buttons");
//            telemetry.addData("Hand Open/Closed", "Left and Right Bumpers");
//            telemetry.addData("-", "-------");

//            telemetry.addData("Drive Power", "%.2f", drive);
//            telemetry.addData("Turn Power",  "%.2f", turn);
//            telemetry.addData("Arm Power",  "%.2f", arm);
//            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
//            telemetry.addData("-", "-------");
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
