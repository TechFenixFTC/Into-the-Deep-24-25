package org.firstinspires.ftc.teamcode.opmode.v1_opModes;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
@Config
@Disabled
@Autonomous(name = "TEST_AUTO", group = "Autonomous")
public class TesteAutonomo extends  LinearOpMode{
    // vision here that outputs position
    public int visionOutputPosition = 1;
    Action trajectoryAction1;
    Action trajectoryAction2;
    Action trajectoryAction3;
    Action trajectoryActionCloseOut;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 70, Math.toRadians(0)));
        MecanumDrive.PARAMS.maxProfileAccel = 80;

        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .waitSeconds(4)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(50, 25), Math.toRadians(0))
                .waitSeconds(1)
                .lineToX(40)
                .build();
        trajectoryAction2 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .splineTo(new Vector2d(50, 25), Math.toRadians(0))
                .waitSeconds(1)
                .lineToX(40)
                .build();
        trajectoryAction3 = drive.actionBuilder(drive.pose)
                .setTangent(0)
                .strafeTo(new Vector2d(50, 25))
                .waitSeconds(1)
                .lineToX(40)
                .build();


        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryAction1;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryAction2;
        } else {
            trajectoryActionChosen = trajectoryAction3;
        }

        Actions.runBlocking(
                new SequentialAction(
                       trajectoryActionChosen
                       // lift.liftUp(),
                       //claw.openClaw(),
                       // lift.liftDown(),

                )

        );
        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .waitSeconds(2)
                .strafeTo(new Vector2d(40, -5))
                .strafeTo(new Vector2d(70, -5))
                .turn(Math.toRadians(180))
                .build();
        Actions.runBlocking(
            new SequentialAction(
                trajectoryActionCloseOut
                // lift.liftUp(),
                //claw.openClaw(),
                // lift.liftDown(),

            )

        );

    }

}
