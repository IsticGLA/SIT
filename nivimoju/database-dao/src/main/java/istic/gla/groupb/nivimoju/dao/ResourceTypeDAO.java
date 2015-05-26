package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by jeremy on 09/04/15.
 */
public class ResourceTypeDAO extends AbstractDAO<ResourceType> {

    public ResourceTypeDAO() {
        this.typeClass = ResourceType.class;
        this.type = Constant.TYPE_RESOURCE_TYPE;
    }
}
