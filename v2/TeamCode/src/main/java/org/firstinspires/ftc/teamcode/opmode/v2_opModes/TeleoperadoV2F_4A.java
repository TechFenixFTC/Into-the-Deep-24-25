package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

@TeleOp(name="Teleoperado V2f")
public class TeleoperadoV2F_4A extends OpMode {


    private V2 robot;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

       robot = this.createRobot(hardwareMap);

    }
    @Override
    public void loop() {
        this.gerenciarModo(robot);
        this.robotCentricDrive(robot);
        this.binds(robot);
        this.linearHorizontal(robot.intakeOutake.linearHorizontal);
        this.linearVertical(robot.intakeOutake.linearVertical);
        this.bracoGarra(robot.intakeOutake.braco);
    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot)  {
    }
    private void binds(V2 robot) {

    }
    private void linearHorizontal(LinearHorizontal horizontal)  {
    }
    private void linearVertical(LinearVertical vertical)  {
    }
    private void bracoGarra(BracoGarra braco)  {
    }
    private void gerenciarModo(V2 robot) {


    }
    private void runActions(OrdersManager carteiro) {}

}

