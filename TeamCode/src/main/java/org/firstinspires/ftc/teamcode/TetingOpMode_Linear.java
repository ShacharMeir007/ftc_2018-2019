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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="Op Mode for testing", group="Linear Opmode")
//@Disabled

public class TetingOpMode_Linear extends OpModeRobot_Linear {
     DriveTrain.Shape shape = DriveTrain.Shape.BALL;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int cm =124;
    private int cmJump =42;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables.
        Shachar.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Shachar.driveTrain.driveForward_PID(37, 0.3);
        Shachar.driveTrain.driveLeft(0.1,90);
        Shachar.driveTrain.driveForward_PID(-32, -0.3);
        for (int i = 1; i <=2 ; i++) {

            /*arm down*/Shachar.driveTrain.lightServo.setPosition(0.0);

                sleep(100);
                shape = Shachar.driveTrain.ShapeCheck(telemetry,runtime);

            if (shape== DriveTrain.Shape.CUBE){
                break;
            }
            else {
                sleep(1000);
                /*arm up*/
                Shachar.driveTrain.lightServo.setPosition(1);
                cm -= cmJump;
                Shachar.driveTrain.driveForward(cmJump, 0.3);
            }
        }
        Shachar.driveTrain.lightServo.setPosition(0.0);
        Shachar.driveTrain.driveForward_PID(20,0.3);
        Shachar.driveTrain.lightServo.setPosition(1);
        Shachar.driveTrain.driveForward_PID(cm-20,0.3);

        telemetry.update();
        runtime.reset();
        /*here you write the autonomous program*/
        shape = Shachar.driveTrain.ShapeCheck(telemetry,runtime);
        switch (shape){
            case BALL:
                print("shape: ","ball");
                break;
            case CUBE:
                Shachar.driveTrain.driveForward_PID(20,0.3);
                print("shape: ","cube");
                break;
            default:
                break;
        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            //print on the app screen.
            telemetry.addData("shape:",shape);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
