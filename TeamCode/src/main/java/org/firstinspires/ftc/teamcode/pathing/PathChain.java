/*
 * An ordered sequence of paths the follower runs end to end. Original
 * implementation for this template.
 */
package org.firstinspires.ftc.teamcode.pathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathChain {
    private final List<Path> paths;

    public PathChain(Path... paths) {
        this.paths = new ArrayList<>(Arrays.asList(paths));
    }

    public PathChain(List<Path> paths) {
        this.paths = new ArrayList<>(paths);
    }

    public Path get(int index) {
        return paths.get(index);
    }

    public int size() {
        return paths.size();
    }

    public boolean isEmpty() {
        return paths.isEmpty();
    }
}
