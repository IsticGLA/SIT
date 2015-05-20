package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;

import java.io.Serializable;

/**
 * Created by vivien on 08/04/15.
 */
public class User extends AbstractEntity implements Serializable {

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

    public String getLogin() {
        return login;
    }

    public void setLogin(String login) {
        this.login = login;
    }

    public String getPassword() {
        return password;
    }

    public void setPassword(String password) {
        this.password = password;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof User)) return false;

        User user = (User) o;

        if (!login.equals(user.login)) return false;
        return password.equals(user.password);

    }

    @Override
    public int hashCode() {
        int result = login.hashCode();
        result = 31 * result + password.hashCode();
        return result;
    }
}
