package org.firstinspires.ftc.teamcode.agregadoras;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.V1_rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.Garra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

import java.util.ArrayList;
import java.util.List;
public class V2 {
    // Attributes
    public org.firstinspires.ftc.teamcode.roadrunner.V1_rr.MecanumDrive md;
    public Telemetry telemetry;
    List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
    public LinearVertical linearVertical;
    public LinearHorizontal linearHorizontal;
    public Garra garra;
    HardwareMap hardwaremap;


    public V2(HardwareMap hardwareMap, Telemetry telemetry) {

        md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        // Controle de leituras
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.linearVertical = new LinearVertical(hardwareMap);
        this.linearHorizontal = new LinearHorizontal(hardwareMap);
        this.garra = new Garra(hardwareMap);

    }

    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }

}
