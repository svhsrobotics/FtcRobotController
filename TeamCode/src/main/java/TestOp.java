import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.standard.LynxSetModuleLEDPatternCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class TestOp extends LinearOpMode {
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
            //
            //
            //
            //
    };

    List<Blinker.Step> createPattern() {
        List<Blinker.Step> steps = new ArrayList<Blinker.Step>();
        steps.add(new Blinker.Step((Color.RED, 500, TimeUnit.MILLISECONDS));
        steps.add(new Blinker.Step((Color.)))
        // We set the LED to be a solid green, interrupted occasionally by a brief off duration.
        int msLivenessLong = 5000;
        int msLivenessShort = 500;
        steps.add(new Blinker.Step(Color.GREEN, msLivenessLong - msLivenessShort, TimeUnit.MILLISECONDS));
        steps.add(new Blinker.Step(Color.BLACK, msLivenessShort, TimeUnit.MILLISECONDS));

        // Then we blink the module address in blue
        int slotsRemaining = LynxSetModuleLEDPatternCommand.maxStepCount-steps.size();
        int blinkCount = Math.min(lynxModule.getModuleAddress(), slotsRemaining/2);
        for (int i = 0; i < blinkCount; i++)
        {
            steps.add(new Blinker.Step(Color.BLUE, msLivenessShort, TimeUnit.MILLISECONDS));
            steps.add(new Blinker.Step(Color.BLACK, msLivenessShort, TimeUnit.MILLISECONDS));
        }

        return steps;
    }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : modules) {
            if (module.getModuleAddress() == LynxConstants.CH_EMBEDDED_MODULE_ADDRESS) {
                // This is the control hub
            } else {
                // This is an expansion hub
            }
        }
    }
}
