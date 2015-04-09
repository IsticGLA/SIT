package dao;


import entity.Intervention;
import util.Constant;

/**
 * Created by vivien on 09/04/15.
 */
public class InterventionDAO extends AbstractDAO<Intervention> {
    public InterventionDAO() {
        this.typeClass = Intervention.class;
        this.type = Constant.TYPE_INTERVENTION;
    }
}
