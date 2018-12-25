package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by shach on 12/18/2018.
 */
@Autonomous(name="OpMode_Blue_Base", group="Linear Opmode")
public class OpMode_Blue_Base extends OpModeRobot_Linear{
    DriveTrain.Shape shape = DriveTrain.Shape.BALL;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private int cm =124;
    private int cmJump =36;
    int i;
    DriveTrain.Place place = DriveTrain.Place.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        Shachar.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /*here you write the autonomous program*/
        /*segment 1*/
        Shachar.driveTrain.driveForward_PID(39, 0.3);
        Shachar.driveTrain.turnRight_ENCODERS(0.17,85);
        Shachar.driveTrain.driveForward_PID( -35, -0.3);
             /*segment 2*/
        for ( i = 0; i <=2 ; i++) {
            Shachar.driveTrain.lightSensorDown();

            sleep(100);
            shape = Shachar.driveTrain.ShapeCheck(telemetry,runtime);

            if (shape== DriveTrain.Shape.CUBE){
                break;

            }
            else {
                sleep(1000);
                /*arm up*/
                Shachar.driveTrain.lightSensorUp();
                cm -= cmJump;
                sleep(100);
                Shachar.driveTrain.driveForward(cmJump, 0.3);
            }

        }
        Shachar.driveTrain.cubeMission(i);
        while (opModeIsActive()) {






            //print on the app screen.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
