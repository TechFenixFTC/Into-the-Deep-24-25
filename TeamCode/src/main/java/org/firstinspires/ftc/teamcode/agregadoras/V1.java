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

public class V1 {
    // Attributes
    public MecanumDrive md;
    public Telemetry telemetry;
    List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
    public LinearVertical linearVertical;
    public LinearHorizontal linearHorizontal;
    public Garra garra;
    HardwareMap hardwaremap;

    public Action GoToBasket;
    public Action GoToSubmersible;
    public V1(HardwareMap hardwareMap, Telemetry telemetry) {

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
        GoToBasket = this.md.actionBuilder(this.md.pose).splineTo(new Vector2d(15, 35), Math.toRadians(130)).build();
        GoToSubmersible = this.md.actionBuilder(this.md.pose).splineTo(new Vector2d(49, 23), Math.toRadians(-92)).build();
    }


    public void updateAutoRoutes() {

        GoToBasket = this.md.actionBuilder(this.md.pose).splineTo(new Vector2d(15, 43), Math.toRadians(130)).build();
        if (this.md.pose.position.y < 25 && this.md.pose.position.x > 40) {
               GoToBasket =  this.md.actionBuilder(this.md.pose).strafeToLinearHeading(new Vector2d(15, 43), Math.toRadians(130)).build();
        }

        GoToSubmersible = this.md.actionBuilder(this.md.pose).splineTo(new Vector2d(49, 23), Math.toRadians(-92)).build();
        if (this.md.pose.position.y < 25 && this.md.pose.position.x > 40) {
            GoToSubmersible =  this.md.actionBuilder(this.md.pose).strafeToLinearHeading(new Vector2d(49, 23), Math.toRadians(-92)).build();
        }
    }
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }


}
