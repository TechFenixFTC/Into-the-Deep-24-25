package org.firstinspires.ftc.teamcode.agregadoras;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.Garra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

import java.util.ArrayList;
import java.util.List;
public class V2 {
    // Attributes
    public MecanumDrive md;

    public Telemetry telemetry;
    List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
    public Vertex intakeOutake;
    HardwareMap hardwaremap;
    public Globals globals;
    public boolean overShoot = false;
    public static boolean teelop = false, risky = false;
    public static double deposit_y = -44, deposit_x = -42;


    public V2(HardwareMap hardwareMap, Telemetry telemetry) {

        md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));




        // -Controle de leituras- \\
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.hardwaremap = hardwareMap;

        this.telemetry = telemetry;
        this.intakeOutake = new Vertex(hardwareMap, telemetry);
        if(getVoltage() > 13.4){
            overShoot = true;
        }

        if(overShoot) {
            //BracoGarra.kp = BracoGarra.kp * 0.7;
            this.intakeOutake.braco.overshoot = true;

        }
    }

    /*
    aqui preciso colocar as ações prontas do chassi (e toda lógica pra isso funcionar) além de ações que envolvem o v2
    todo: gotoBasket
    todo: goToSubmersible
    todo: goNearTheObservationZone
     */

    public Action goDepositRedBasket(String tipo) {
        this.md.updatePoseEstimate();
        if (tipo.equals("strafe")) {
            if(risky) {
                return new SequentialAction(
                        new InstantAction(() -> {MecanumDrive.PARAMS.kV *= 5;}),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kA /= 2;}),

                        new ParallelAction(
                                this.intakeOutake.linearVertical.ElevadorGoTo(2800),
                                this.intakeOutake.braco.goToBasketOutake(0, 0),
                                this.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                                this.md.actionBuilder(this.md.pose)
                                        .setTangent(Math.toRadians(90))
                                        .strafeToLinearHeading(new Vector2d(deposit_x,deposit_y), Math.toRadians(-135))

                                        .build()

                        ),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kV /= 5;}),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kA *= 2;})

                );
            }
            return new SequentialAction(
                    new ParallelAction(
                            this.intakeOutake.linearVertical.ElevadorGoTo(2800),

                            this.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                            this.md.actionBuilder(this.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-53, -58), Math.toRadians(-135))
                                    .build()

                    ),
                    this.intakeOutake.braco.goToBasketOutake(0, 0)

            );
        }
        return new SequentialAction(
                new ParallelAction(
                        this.intakeOutake.linearVertical.ElevadorGoTo(2800),
                        this.intakeOutake.braco.goToBasketOutake(0, 0),
                        this.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                        this.md.actionBuilder(this.md.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-48, -45,  Math.toRadians(-135)), Math.toRadians(-135))

                                .build()

                )
        );
    }

    public Action goToRedBasket(String tipo) {
        this.md.updatePoseEstimate();
        if (tipo.equals("strafe")) {
            return new SequentialAction(
                    new ParallelAction(
                            this.intakeOutake.linearVertical.ElevadorGoTo(2800),
                            this.intakeOutake.braco.goToBasketOutake(0, 0),
                            this.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                            this.md.actionBuilder(this.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-38, -29), Math.toRadians(-135)).build(),
                                    new SequentialAction(
                                            this.md.actionBuilder(this.md.pose).waitSeconds(2).build()

                                    )


                    )

            );
        }
        return new SequentialAction(
                new ParallelAction(
                        this.intakeOutake.linearVertical.ElevadorGoTo(2800),
                        new InstantAction(() -> {this.intakeOutake.braco.autonomo = true;}),
                        this.intakeOutake.braco.goTo150(0, 0),
                        this.intakeOutake.braco.goToStored(0, 0),
                        this.intakeOutake.braco.goToBasketOutake(0, 0),
                        new InstantAction(() -> {this.intakeOutake.braco.autonomo = false;}),
                        this.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                        this.md.actionBuilder(this.md.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-35, -25,  Math.toRadians(-135)), Math.toRadians(-135)).build(),
                        new SequentialAction(
                                this.md.actionBuilder(this.md.pose).waitSeconds(2).build(),
                                new InstantAction(() -> {MecanumDrive.PARAMS.kV /= 5;}),
                                this.intakeOutake.linearHorizontal.extenderNemTanto(0, 0),
                                new InstantAction(() -> {MecanumDrive.PARAMS.kV *= 5;})
                        )

                )
        );
    }
    public Action goToRedSubmersible(String tipo) {
        this.md.updatePoseEstimate();
        if (tipo.equals("strafe")) {
            return new SequentialAction(
                    new ParallelAction(
                            this.intakeOutake.linearVertical.ElevadorGoTo(2800),
                            this.intakeOutake.braco.goToBasketOutake(0, 0),
                            this.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                            this.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                            this.md.actionBuilder(this.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-38, -29), Math.toRadians(-135)).build(),
                            new SequentialAction(
                                    this.md.actionBuilder(this.md.pose).waitSeconds(2).build(),
                                    this.intakeOutake.linearHorizontal.extender(0, 0)
                            )


                    )

            );
        }
        return new SequentialAction(
                new ParallelAction(
                    new SequentialAction(
                    this.md.actionBuilder(this.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-8, 6,  Math.toRadians(0)), Math.toRadians(0))
                        .build()

                )
                )
        );
    }
    public double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }
    public Action sensorMovimentation() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (intakeOutake.garra.colorSensor.alpha() < 100) {

                    md.leftBack.setPower(0.3);
                    md.leftFront.setPower(0.3);
                    md.rightFront.setPower(0.3);
                    md.rightBack.setPower(0.3);

                    return true;
                }


                if (intakeOutake.garra.colorSensor.alpha() > 160 && intakeOutake.garra.colorSensor.alpha() < 260) {
                    md.leftBack.setPower(0);
                    md.leftFront.setPower(0);
                    md.rightFront.setPower(0);
                    md.rightBack.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

}
