/*
 * Serves the visual path planner from the Robot Controller's embedded web
 * server so it is reachable at the robot's IP (port 8080), e.g.
 *   http://192.168.43.1:8080/pathplanner
 *
 * Registration uses the FTC SDK's @WebHandlerRegistrar hook, which the SDK
 * calls once at startup with the app context and the web-handler manager. The
 * handler streams assets/pathplanner.html. Original implementation for this
 * template.
 *
 * NOTE: this depends on SDK web-server internals that can only be exercised on
 * real hardware (or the FTC SDK runtime). Verify on the robot after deploying:
 * connect to the robot's network and browse to /pathplanner on port 8080.
 */
package org.firstinspires.ftc.teamcode.pathing;

import android.content.Context;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

import fi.iki.elonen.NanoHTTPD;

public final class PathPlannerServer {
    private PathPlannerServer() {}

    private static final String ASSET_NAME = "pathplanner.html";
    private static final String ROUTE = "/pathplanner";

    /**
     * Called automatically by the SDK at startup. Registers the planner route.
     * The method must be public static and take (Context, WebHandlerManager).
     */
    @WebHandlerRegistrar
    public static void register(Context context, WebHandlerManager manager) {
        final AssetManager assets = context.getAssets();
        manager.register(ROUTE, new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session)
                    throws IOException, NanoHTTPD.ResponseException {
                try {
                    String html = readAsset(assets, ASSET_NAME);
                    NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK, "text/html; charset=utf-8", html);
                    response.addHeader("Cache-Control", "no-store");
                    return response;
                } catch (IOException e) {
                    return NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.NOT_FOUND, "text/plain",
                            ASSET_NAME + " not found in assets");
                }
            }
        });
    }

    private static String readAsset(AssetManager assets, String name) throws IOException {
        try (InputStream in = assets.open(name)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[4096];
            int read;
            while ((read = in.read(buffer)) != -1) {
                out.write(buffer, 0, read);
            }
            return out.toString("UTF-8");
        }
    }
}
