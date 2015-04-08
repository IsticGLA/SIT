package entity;


import util.Constant;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by arno on 09/03/15.
 */
public class Intervention extends AbstractEntity implements Serializable {

    public Intervention() {
        super();
        this.type = Constant.TYPE_INTERVENTION;
    }

}


