package dao;

import entity.StaticData;
import util.Constant;

/**
 * Created by jeremy on 09/04/15.
 */
public class StaticDataDAO extends AbstractDAO<StaticData> {

    public StaticDataDAO() {
        this.typeClass = StaticData.class;
        this.type = Constant.TYPE_STATIC_DATA;
    }
}
