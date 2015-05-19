package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by jeremy on 19/05/15.
 */
public class ImageDAO extends AbstractDAO<Image> {

    public ImageDAO() {
        this.typeClass = Image.class;
        this.type = Constant.TYPE_IMAGE;
    }
}
