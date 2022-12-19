import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard;
import org.firstinspires.ftc.teamcode.util.Blinker;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Toggle;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class Rainbow extends LinearOpMode {
    int[] colors = {
            Color.rgb(255, 0,   0),   // RED
            Color.rgb(255, 128, 0),   // ORANGE
            Color.rgb(255, 255, 0),   // YELLOW
            Color.rgb(128, 255, 0),   // LIME
            Color.rgb(0,   255, 0),   // GREEN
            Color.rgb(0,   255, 128), // MINT
            Color.rgb(0,   255, 255), // CYAN
            Color.rgb(0,   128, 255), // BLUE
            Color.rgb(0,   0,   255), // NAVY
            Color.rgb(128, 0,   255), // PURPLE
            Color.rgb(255, 0,   255), // PINK
            Color.rgb(255, 0,   128), // HOT PINK
    };


    Blinker.Pattern rainbowPattern() {
        Blinker.Pattern pattern = new Blinker.Pattern();
        for (int color : colors) {
            pattern.addStep(color, 150);
        }
        return pattern;
    }

    LynxModule controlHub(List<LynxModule> modules) {
        for (LynxModule module : modules) {
            if (module.getModuleAddress() == LynxConstants.CH_EMBEDDED_MODULE_ADDRESS) {
                return module;
            }
        }
        return null;
    }

    Debouncer debouncer = new Debouncer();
    boolean last = false;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        LynxModule chub = controlHub(modules);
        Blinker blinker = new Blinker(chub);
        Logger logger = new Logger(telemetry);
        //Blinker.Pattern rainbow = rainbowPattern();

        waitForStart();

        DigitalChannel button = AndroidBoard.getInstance().getUserButtonPin();
        while (!isStopRequested()) {
            if (debouncer.update(button.getState())) {
                if (last == false) {
                    blinker.displayPattern(rainbowPattern());
                    last = true;
                } else {
                    blinker.idle();
                    last = false;
                }
            }
        }
    }
}
