import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.util.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

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

    List<Blinker.Step> rainbowPattern() {
        List<Blinker.Step> steps = new ArrayList<>();
        for (int color : colors) {
            steps.add(new Blinker.Step(color, 500, TimeUnit.MILLISECONDS));
        }
        return steps;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        Logger logger = new Logger(telemetry);
        List<Blinker.Step> rainbow = rainbowPattern();

        for (LynxModule module : modules) {
            if (module.getModuleAddress() == LynxConstants.CH_EMBEDDED_MODULE_ADDRESS) {
                // This is the control hub
                logger.info("Found a Control Hub");
            } else {
                // This is an expansion hub
                logger.info("Found an Expansion Hub with address: " + module.getModuleAddress());
            }
            module.setPattern(rainbow);
        }
    }
}
