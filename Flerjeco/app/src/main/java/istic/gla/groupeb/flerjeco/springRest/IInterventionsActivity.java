package istic.gla.groupeb.flerjeco.springRest;

import entity.Intervention;

/**
 * Created by jules on 16/04/15.
 */
public interface IInterventionsActivity {

    void showProgress(final boolean show);

    void updateInterventions(Intervention[] interventions);
}
