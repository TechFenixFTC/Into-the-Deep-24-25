package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.agregadoras.Vertex;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.LinearVertical;

@TeleOp(name="Teleoperado V2f")
public class TeleoperadoV2F_4A extends OpMode {


    private V2 robot;
    GamepadEx gamepadEx;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx = new GamepadEx(gamepad2);
        robot.intakeOutake.RobotState = Vertex.vertexState.Initial;
        Globals.IS_STORED = true;


    }
    @Override
    public void loop() {
        this.gerenciarModo(robot);
        this.robotCentricDrive(robot);
        this.binds(robot);
        this.linearHorizontal(robot.intakeOutake.linearHorizontal);
        this.linearVertical(robot.intakeOutake.linearVertical);
        this.bracoGarra(robot.intakeOutake.braco);
        this.runActions(robot.intakeOutake.carteiro);
        telemetry.addData("Alvo",robot.intakeOutake.braco.targetPosition);
        telemetry.addData("Atual",robot.intakeOutake.braco.motorBracoGarra.getCurrentPosition());

    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot)  {

    }
    private void binds(V2 robot) {

        if(gamepadEx.getButton(GamepadKeys.Button.A) || Globals.IS_INTAKE){
            robot.intakeOutake.IntermediatePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intermediate;


            Globals.IS_INTAKE = true;
            Globals.IS_SCORING = false;
            Globals.IS_STORED = false;
            Globals.IS_ARMED = false;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.B) || Globals.IS_STORED){
            robot.intakeOutake.InitialPosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Initial;

            Globals.IS_INTAKE = false;
            Globals.IS_SCORING = false;
            Globals.IS_STORED = true;
            Globals.IS_ARMED = false;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.X) || Globals.IS_ARMED ){
            robot.intakeOutake.IntakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intake;

            Globals.IS_INTAKE = false;
            Globals.IS_SCORING = false;
            Globals.IS_STORED = false;
            Globals.IS_ARMED = true;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.Y) || Globals.IS_SCORING){
            robot.intakeOutake.OuttakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Outtake;

            Globals.IS_INTAKE = false;
            Globals.IS_SCORING = true;
            Globals.IS_STORED = false;
            Globals.IS_ARMED = false;
        }
    }
    private void linearHorizontal(LinearHorizontal horizontal)  {
    }
    private void linearVertical(LinearVertical vertical)  {
    }
    private void bracoGarra(BracoGarra braco)  {
        if(gamepad2.dpad_left){
            robot.intakeOutake.braco.upBraco(5);
        }
        if(gamepad2.dpad_right ){
            robot.intakeOutake.braco.downBraco(5);
        }
        robot.intakeOutake.braco.handleBracoTeleop(getRuntime());
    }
    private void gerenciarModo(V2 robot) {


    }
    private void runActions(OrdersManager carteiro) {

        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
        carteiro.removeOrderByName(robot.intakeOutake.getRobotState().name());

    }

}