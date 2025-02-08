package org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior.SubsistemasInferiores;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior.SubsistemasSuperiores;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;

import java.util.ArrayList;
import java.util.List;

public class V5 {
    // Attributes
    public MecanumDrive md;
    public  static double
            eixoX = 0,
            eixoY = -30;


    public Telemetry telemetry;
    public OrdersManager carteiro;
    public V5Modes v5Modes = V5Modes.SPECIMEN;
    List<Encoder> leftEncs =  new ArrayList<>(), rightEncs = new ArrayList<>();

    public SubsistemasInferiores intakeInferior;
    public SubsistemasSuperiores outtakeIntakeSuperior;
    HardwareMap hardwaremap;

    public static boolean teelop = false, risky = false;
    public static double deposit_y = -44, deposit_x = -42;


    public V5(HardwareMap hardwareMap, Telemetry telemetry) {

        md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // -Controle de leituras- \\
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.intakeInferior = new SubsistemasInferiores(hardwareMap, telemetry);
        this.outtakeIntakeSuperior = new SubsistemasSuperiores(hardwareMap, telemetry);
        carteiro = new OrdersManager(telemetry);
    }

    /*
    aqui preciso colocar as ações prontas do chassi (e toda lógica pra isso funcionar) além de ações que envolvem o v2
    todo: gotoBasket
    todo: goToSubmersible
    todo: goNearTheObservationZone
     */
    public double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }
    public Action sensorMovimentation() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                /*if (intakeOutake.garra.colorSensor.alpha() < 100) {

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
                }*/
                return true;
            }
        };
    }
    public Action MoveOuttake(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeToConstantHeading(new Vector2d(eixoX, eixoY))
                        .build()
        );
    }
    public Action MoveSpline(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .splineTo(new Vector2d(eixoX, eixoY),Math.toRadians(-90))
                        .build()
        );
    }

    public Action MoveIntake(V5 robot){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeToConstantHeading(new Vector2d(38, -60))
                        .build()
        );
    }

    public Action DiseablePSESuperior(V5 robot, double runtime){
        return new Action() {
            ElapsedTime time = new ElapsedTime();
            boolean FIRST = true;
            double delay = 1;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(FIRST) {
                    time.reset();
                    FIRST = false;
                }
                if(robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getController().getPwmStatus()== ServoController.PwmStatus.ENABLED) {
                    return false;
                }
                    if (robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.READY_TOINTAKE) {
                        double cooldown = delay + time.time();
                        if (time.time() < cooldown) {
                            carteiro.addOrder(robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(), 0.0, "braco superior", runtime);
                            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeCHAMBER(), 0.0, "garra superior", runtime);
                            carteiro.addOrder(robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(700), 0.0, "linear vertical", runtime);
                            // return true;
                        }

                }
                    robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getController().pwmDisable();
                    robot.outtakeIntakeSuperior.garraSuperior.servoRotacaoDaGarra.getController().pwmDisable();
                    robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.getController().pwmDisable();
                    robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.getController().pwmDisable();
                    robot.outtakeIntakeSuperior.linearVertical.motorR.setPower(0);
                    robot.outtakeIntakeSuperior.linearVertical.motorL.setPower(0);
                    return false;

            }
        };
    }
}
