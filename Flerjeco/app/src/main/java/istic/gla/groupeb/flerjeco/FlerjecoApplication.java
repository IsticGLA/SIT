package istic.gla.groupeb.flerjeco;

import android.app.Application;
import android.content.Context;

import istic.gla.groupb.nivimoju.entity.IncidentCode;
import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupeb.flerjeco.springRest.IStaticDataActivity;

/**
 * Created by corentin on 09/04/15.
 */
public class FlerjecoApplication extends Application implements IStaticDataActivity {

    private boolean isCodisUser; //make getter and setter
    private String login, password;
    private StaticData[] staticData;
    private static FlerjecoApplication singleInstance = null;
    private ResourceType[] resourceTypes;
    private IncidentCode[] incidentCodes;

    public static FlerjecoApplication getInstance()
    {
        return singleInstance;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        singleInstance = this;
    }

    public boolean isCodisUser() {
        return isCodisUser;
    }

    public void setCodisUser(boolean isCodisUser) {
        this.isCodisUser = isCodisUser;
    }

    public String getPassword() {
        return password;
    }

    public void setPassword(String password) {
        this.password = password;
    }

    public String getLogin() {
        return login;
    }

    public void setLogin(String login) {
        this.login = login;
    }

    public StaticData[] getStaticData() {
        return staticData;
    }

    public void setStaticData(StaticData[] staticData) {
        this.staticData = staticData;
    }

    @Override
    public Context getContext() { return getContext(); }

    public void setResourceTypes(ResourceType[] resourceTypes) {
        this.resourceTypes = resourceTypes;
    }

    public ResourceType[] getResourceTypes() {
        return resourceTypes;
    }

    public void setIncidentCodes(IncidentCode[] incidentCodes) {
        this.incidentCodes = incidentCodes;
    }

    public IncidentCode[] getIncidentCodes() {
        return incidentCodes;
    }
}
