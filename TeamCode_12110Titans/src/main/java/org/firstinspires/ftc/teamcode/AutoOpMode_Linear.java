/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

/**
*hi
*/

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Yeet Mobile", group="Titans Linear Opmode")
public class AutoOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor latcherFar = null;
    private DcMotor latcherClose = null;
    private DcMotor armyDude = null;
    private DcMotor armyOff = null;
private Servo mrclawPants = null;
private Servo mrsclawPants = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //mrsclawPants doesn't exist.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        latcherFar = hardwareMap.get (DcMotor.class, "latcher_far");
        latcherClose = hardwareMap.get (DcMotor.class, "latcher_close");
        armyDude = hardwareMap.get (DcMotor.class, "army_dude");
        armyOff = hardwareMap.get (DcMotor.class, "army_off");
        mrclawPants = hardwareMap.get (Servo.class, "mrclaw_pants");
        mrsclawPants = hardwareMap.get (Servo.class, "mrsclaw_pants");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        latcherFar.setDirection(DcMotor.Direction.FORWARD);
        latcherClose.setDirection(DcMotor.Direction.REVERSE);
        armyDude.setDirection(DcMotor.Direction.FORWARD);
        armyOff.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latcherFar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        latcherClose.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armyDude.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armyOff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

// Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double leftTriggerPower;
        double rightTriggerPower;
        double armyDudePower;
        double armyOffPower;
        double mrclawPos;
        double mrsclawPos;

        //set up an auto start trigger to set opModeIsActive to True


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // TODO: all of these values need to be checked this is position before start
            double lPower = 0.0;   // left motion motor
            double rPower = 0.0;   // right motion motor
            double ltPower = 0.5;  // latch motor 1 position at start
            double rtPower = 0.5;  // latch motor 2 position at start
            double adPower = 0.0;  // arm motor 1 position at start
            double aoPower = 0.0;  // arm motor 2 position at start
            double mrcPos = 0.0; // claw position at start
            double mrscPos = 1.0; // claw position at start
            armyDudePower  = Range.clip(adPower, -1.0, 1.0);
            armyOffPower  = Range.clip(aoPower, -1.0, 1.0);
            armyDude.setPower(armyDudePower);
            armyOff.setPower(armyOffPower);
            mrclawPos  = Range.clip(mrcPos, -0.5, 0.5);
            mrsclawPos  = Range.clip(mrscPos, -0.5, 0.5);
            mrclawPants.setPosition(mrclawPos);
            mrsclawPants.setPosition(mrsclawPos);

            // Put robot on ground
            ltPower = 0.0; // position to fully extend motor 1
            rtPower = 0.0; // position to fully extend motor 2
            leftTriggerPower  = Range.clip(ltPower, -1.0, 1.0);
            rightTriggerPower  = Range.clip(rtPower, -1.0, 1.0);
            latcherFar.setPower(leftTriggerPower);
            latcherClose.setPower(rightTriggerPower);
            telemetry.addData("Motors", "leftTrigger (%.2f), rightTrigger (%.2f)", leftTriggerPower, rightTriggerPower);
            telemetry.update();


            //de-latch
            rtPower = -1.0; // position to fully extend motor 2
            rightTriggerPower  = Range.clip(rtPower, -1.0, 1.0);
            latcherClose.setPower(rightTriggerPower);
            telemetry.addData("Motors", "leftTrigger (%.2f), rightTrigger (%.2f)", leftTriggerPower, rightTriggerPower);
            telemetry.update();


            // retract latch
            ltPower = 1.0; // position to un-extend motor 1
            leftTriggerPower  = Range.clip(ltPower, -1.0, 1.0);
            latcherFar.setPower(leftTriggerPower);
            telemetry.addData("Motors", "leftTrigger (%.2f), rightTrigger (%.2f)", leftTriggerPower, rightTriggerPower);
            telemetry.update();

            // Turn Right
            lPower = 1.0;   // left motion motor
            rPower = -1.0;   // right motion motor
            leftPower    = Range.clip(lPower, -1.0, 1.0) ;
            rightPower   = Range.clip(rPower, -1.0, 1.0) ;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            sleep(5);
            lPower = 0.0;   // left motion motor
            rPower = 0.0;   // right motion motor
            leftPower    = Range.clip(lPower, -1.0, 1.0) ;
            rightPower   = Range.clip(rPower, -1.0, 1.0) ;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            // move forward (if required)
            lPower = 1.0;   // left motion motor
            rPower = 1.0;   // right motion motor
            leftPower    = Range.clip(lPower, -1.0, 1.0) ;
            rightPower   = Range.clip(rPower, -1.0, 1.0) ;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            sleep(5);
            lPower = 0.0;   // left motion motor
            rPower = 0.0;   // right motion motor
            leftPower    = Range.clip(lPower, -1.0, 1.0) ;
            rightPower   = Range.clip(rPower, -1.0, 1.0) ;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            // Find gold and silver items

            // else turn right or left to find gold block

            // move only the gold block (pick up with arm?)

            // find the crater picture

            // scan the crater picture

            // based on the scan determine team color and where you are in the field

            // move to the correct color box

            // set claim flag ????


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors", "leftTrigger (%.2f), rightTrigger (%.2f)", leftTriggerPower, rightTriggerPower);
            telemetry.addData("Motors","armyDude (%.2f), armyOff (%.2f)", armyDudePower, armyOffPower);
            telemetry.addData("Motors", "mrclawPants (%.2f), mrsclawPants", mrclawPos,mrsclawPos);
            telemetry.update();

        }
    }
}
