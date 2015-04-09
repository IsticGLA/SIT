package dao;

import entity.IncidentCode;
import util.Constant;

/**
 * Created by jeremy on 09/04/15.
 */
public class IncidentCodeDAO extends AbstractDAO<IncidentCode> {

    public IncidentCodeDAO() {
        this.typeClass = IncidentCode.class;
        this.type = Constant.TYPE_INCIDENT_CODE;
    }
}
