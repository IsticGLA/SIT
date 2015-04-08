package entity;

import util.Constant;

/**
 * Created by vivien on 08/04/15.
 */
public class User extends AbstractEntity {

    private String login;
    private String password;

    /**
     * Build a User
     */
    public User() {
        super();
        this.type = Constant.TYPE_USER;
    }

    /**
     * Build a User
     * @param login
     * @param password
     */
    public User(String login, String password) {
        super();
        this.type = Constant.TYPE_USER;
        this.login = login;
        this.password = password;
    }
}
