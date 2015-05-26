package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.User;
import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by vivien on 08/04/15.
 */
public class UserDAO extends AbstractDAO<User> {

    public UserDAO() {
        this.typeClass = User.class;
        this.type = Constant.TYPE_USER;
    }
}
