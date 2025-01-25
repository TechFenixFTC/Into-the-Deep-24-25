package org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Subsistemas;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
@Deprecated
public class V2 {
   /*
    // Attributes
    public MecanumDrive md;

    public Telemetry telemetry;
    List<Encoder> leftEncs =  new ArrayList<>(), rightEncs = new ArrayList<>();
    public Subsistemas intakeOutake;
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
        this.intakeOutake = new Subsistemas(hardwareMap, telemetry);
        if(getVoltage() > 13.4){
            overShoot = true;
        }

        if(overShoot) {
            //BracoGarra.kp = BracoGarra.kp * 0.7;
            //this.intakeOutake.braco.overshoot = true;

        }
    }


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
                }
                return true;
            }
        };
    }
*/
}
