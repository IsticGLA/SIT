package istic.gla.groupeb.flerjeco.agent.intervention;

import android.animation.Animator;
import android.animation.AnimatorListenerAdapter;
import android.annotation.TargetApi;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.Spinner;

import entity.Intervention;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;

/**
 * Fragment used for change the state of a resource of firefighters
 * @see  istic.gla.groupeb.flerjeco.codis.intervention.OnTaskCompleted
 * @see android.support.v4.app.DialogFragment
 */
public class ChangeStateDialogFragment extends DialogFragment {
    private static final String TAG = ChangeStateDialogFragment.class.getSimpleName();

    Spinner stateSpinner;

    //fields
    Button validateButton;
    Button freeButton;

    private View mProgressView;
    private View mCreateFormView;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_change_resource, container, false);
        getDialog().setTitle(R.string.title_fragment_change_resource);

        mCreateFormView = v.findViewById(R.id.intervention_scroll);
        mProgressView = v.findViewById(R.id.create_progress);


        //init fields
        validateButton = (Button) v.findViewById(R.id.validateButton);
        freeButton = (Button) v.findViewById(R.id.freeButton);

        //init state spinner
        stateSpinner = (Spinner) v.findViewById(R.id.stateSpinner);



        //Selection of registered claims codes in the database and the list of identifiers of resourceType
        showProgress(true);


        return v;
    }

    /**
     * Shows the progress UI and hides the login form.
     */
    @TargetApi(Build.VERSION_CODES.HONEYCOMB_MR2)
    public void showProgress(final boolean show) {
        // On Honeycomb MR2 we have the ViewPropertyAnimator APIs, which allow
        // for very easy animations. If available, use these APIs to fade-in
        // the progress spinner.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB_MR2) {
            int shortAnimTime = getResources().getInteger(android.R.integer.config_shortAnimTime);

            mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
            mCreateFormView.animate().setDuration(shortAnimTime).alpha(
                    show ? 0 : 1).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
                }
            });

            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mProgressView.animate().setDuration(shortAnimTime).alpha(
                    show ? 1 : 0).setListener(new AnimatorListenerAdapter() {
                @Override
                public void onAnimationEnd(Animator animation) {
                    mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
                }
            });
        } else {
            // The ViewPropertyAnimator APIs are not available, so simply show
            // and hide the relevant UI components.
            mProgressView.setVisibility(show ? View.VISIBLE : View.GONE);
            mCreateFormView.setVisibility(show ? View.GONE : View.VISIBLE);
        }
    }

    @Override
    public void onPause() {
        super.onPause();

    }


}
