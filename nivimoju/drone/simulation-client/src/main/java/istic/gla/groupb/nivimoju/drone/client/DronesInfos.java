package istic.gla.groupb.nivimoju.drone.client;

import java.util.List;

/**
 * containers of inforamtions about multiples drones
 */
public class DronesInfos {
    private List<DroneInfo> infos;

    public List<DroneInfo> getInfos() {
        return infos;
    }

    public void setInfos(List<DroneInfo> infos) {
        this.infos = infos;
    }

    @Override
    public String toString() {
        return "DronesInfos{" +
                "infos=" + infos +
                '}';
    }
}
