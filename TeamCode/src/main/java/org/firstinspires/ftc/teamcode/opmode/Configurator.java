package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Toggle;

import java.io.File;
import java.util.Objects;

@TeleOp

public class Configurator extends LinearOpMode {
    public static Configuration load() {
        return load("config.json");
    }

    public static void save(Configuration config) {
        save(config, "config.json");
    }


    /**
     * Loads the configuration from the JSON file
     * Note that if no configuration file exists, a new one will be created automatically.
     */
    private static Configuration load(String filename) {
        Gson gson = new GsonBuilder()
                .setPrettyPrinting()
                .create();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        if (!file.exists()) {
            Configuration config = new Configuration();
            save(config, filename);
            return config;
        }
        return gson.fromJson(ReadWriteFile.readFile(file), Configuration.class);
    }

    /**
     * Saves the configuration to the JSON file
     */
    private static void save(Configuration config, String filename) {
        Gson gson = new GsonBuilder()
                .setPrettyPrinting()
                .create();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, gson.toJson(config));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Toggle purpleToggle = new Toggle();
        Debouncer innerPark = new Debouncer();
        Debouncer outerPark = new Debouncer();
        Debouncer placePixel = new Debouncer();

        //String park = "none";

        Configuration config = Configurator.load();

        purpleToggle.state = config.placePixel;

//        Toggle innerParkToggle = new Toggle();
//        Toggle outerParkToggle = new Toggle();
//        Toggle placePixelToggle = new Toggle();

        while (!isStopRequested()) {
            purpleToggle.update(gamepad1.a);

            telemetry.addData("purple pixel", purpleToggle.state);

            config.placePixel = purpleToggle.state;

            if (innerPark.update(gamepad1.b)) {
                config.park = "inner";
            } else if (outerPark.update(gamepad1.y)) {
                config.park = "outer";
            } else if (placePixel.update(gamepad1.x)) {
                config.park = "board";
            } else if (gamepad1.dpad_down) {
                config.park = "none";
            }

            telemetry.addData("park", config.park);

            telemetry.update();
        }

        Configurator.save(config);
    }


}
