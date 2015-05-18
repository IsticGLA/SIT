package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import entity.IncidentCode;

/**
 * Created by corentin on 16/04/15.
 */
public interface IIncidentCode {
    void updateIncidentCodes(IncidentCode[] incidentCodes);

    Context getContext();
}
