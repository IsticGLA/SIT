package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import entity.Intervention;

/**
 * Created by jules on 16/04/15.
 */
public interface IInterventionsActivity {

    void updateInterventions(Intervention[] interventions);

    Context getContext();
}