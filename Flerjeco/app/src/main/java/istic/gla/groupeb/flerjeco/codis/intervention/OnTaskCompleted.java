package istic.gla.groupeb.flerjeco.codis.intervention;

import java.util.List;

import entity.ResourceType;

/**
 * Created by amhachi on 13/04/15.
 */

/**
 * The interface to be implemented by activities whom need getting a response from an AsynchTask
 * @see istic.gla.groupeb.flerjeco.codis.intervention.InterventionDialogFragment
 *
 */
public interface OnTaskCompleted {
    void onTaskCompleted(List<ResourceType> resources);
}
