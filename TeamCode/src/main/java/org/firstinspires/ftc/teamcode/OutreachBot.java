/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy ConceptExternalHardwareClass.java into TeamCode (using Android Studio or OnBotJava) then RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 *
 * For comparison purposes, this sample and its accompanying hardware class duplicates the functionality of the
 * RobotTelopPOV_Linear OpMode.  It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * View the RobotHardware.java class file for more details
 *
 *  Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 *  In OnBot Java, add a new OpMode, select this sample, and select TeleOp.
 *  Also add another new file named RobotHardware.java, select the sample with that name, and select Not an OpMode.
 */

@TeleOp(name="Outreach Bot", group="Robot")
public class OutreachBot extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot       = new RobotHardware(this);


    public static final double CLAW_OPEN  = 0.5;//might change depending on tests
    public static final double CLAW_CLOSE  = 0;
    public static final double BUCKET_COLLECT  = 0;
    public static final double BUCKET_DUMP  = 0.8;

    public static final double INTAKE_SPEED = 0.5; //adjust later
    // public static final double ARM_SPEED = 0.3; //add

    double arm2Offset = 0;

    public static final double ARM2_MID_SERVO   =  0.5 ;
    public static final double ARM2_SPEED  = 0.02 ;

    @Override
    public void runOpMode() {
        double drive        = 0;
        double turn         = 0;
//        double handOffset   = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick y axis moves the robot fwd and back, the Left stick x axis turns left and right.
            // This way it's also easy to just drive straight, or just turn.
           // drive = -gamepad1.left_stick_x;
           // turn  =  gamepad1.right_stick_y;

            robot.leftDrive.setPower(gamepad1.left_stick_y); //tank drive where each joystick controls one side.
            robot.rightDrive.setPower(gamepad1.right_stick_y);


            // Combine drive and turn for blended motion. Use RobotHardware class
           // driveRobot(drive, turn);

            //consult drivers
            //one drive in charge of driving and sample arm.
            //one control bucket and specimen arm.

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.dpad_up) //adjust
                arm2Offset += ARM2_SPEED;
            else if (gamepad2.dpad_down)
                arm2Offset -= ARM2_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            arm2Offset = Range.clip(arm2Offset, -0.5, 0.5); //mid servo is .5
            robot.arm2.setPosition(ARM2_MID_SERVO + arm2Offset);

            if (gamepad2.y)
                robot.arm2Intake.setPosition(CLAW_OPEN);
            else if (gamepad2.a)
                robot.arm2Intake.setPosition(CLAW_CLOSE);


            robot.arm1.setPower(gamepad2.right_stick_y); //multiply ARM_SPEED if necessary

            robot.arm1Intake.setPower(gamepad2.left_trigger-gamepad2.right_trigger); //maybe switch left and right based on test
            //dont press both at the same time, since it wont move.

            robot.liftMotor.setPower(gamepad2.left_stick_y); //make it adjustable so that heights can be accurate. change to dpad.

            if (gamepad2.x)
                robot.bucket.setPosition(BUCKET_COLLECT);
            else if (gamepad2.b)
                robot.bucket.setPosition(BUCKET_DUMP);


            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick"); //doesn't really matter: only sends instructions to driver station.

            telemetry.addData("Drive Power", "%.2f", drive); //references drive and turn, making it more helpful since it tells you things you don't know
            telemetry.addData("Turn Power",  "%.2f", turn);
//            telemetry.addData("Arm Power",  "%.2f", arm);
//            telemetry.addData("Hand Position",  "Offset = %.2f", handOffset);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        robot.leftDrive.setPower(leftWheel);
        robot.rightDrive.setPower(rightWheel);
    }
    public void driveRobot(double Drive, double Turn) {
        // Combine drive and turn for blended motion.
        double left  = Drive + Turn*0.7; // avoiding excessive turn speed
        double right = Drive - Turn*0.7;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(left, right);
    }
}
