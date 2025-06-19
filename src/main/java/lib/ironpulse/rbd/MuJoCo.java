package lib.ironpulse.rbd;

import org.mujoco.MuJoCoModelManager;

import java.io.File;

public class MuJoCo {
    private final MuJoCoModelManager modelManager;

    public MuJoCo(File xmlFile) {
        modelManager = new MuJoCoModelManager(xmlFile);
    }
}
