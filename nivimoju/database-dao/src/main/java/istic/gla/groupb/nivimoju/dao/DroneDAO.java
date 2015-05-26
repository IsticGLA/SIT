package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by jeremy on 14/04/15.
 */
public class DroneDAO extends AbstractDAO<Drone> {

    public DroneDAO() {
        this.typeClass = Drone.class;
        this.type = Constant.TYPE_DRONE;
    }
}