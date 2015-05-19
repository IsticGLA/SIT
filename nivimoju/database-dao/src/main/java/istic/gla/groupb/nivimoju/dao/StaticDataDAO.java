package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by jeremy on 09/04/15.
 */
public class StaticDataDAO extends AbstractDAO<StaticData> {

    public StaticDataDAO() {
        this.typeClass = StaticData.class;
        this.type = Constant.TYPE_STATIC_DATA;
    }
}
