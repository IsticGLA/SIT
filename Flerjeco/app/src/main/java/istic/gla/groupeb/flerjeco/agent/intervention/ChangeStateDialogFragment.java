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
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.Spinner;

import entity.Intervention;
import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.synch.DisplaySynch;
import istic.gla.groupeb.flerjeco.synch.IntentWraper;
import util.ResourceRole;
import util.State;

/**
 * Fragment used for change the state of a resource of firefighters
 * @see android.support.v4.app.DialogFragment
 */
public class ChangeStateDialogFragment extends DialogFragment {
    private static final String TAG = ChangeStateDialogFragment.class.getSimpleName();

    Spinner stateSpinner;
    Resource resource;
    private ResourceRole role;

    //fields
    Button validateButton;
    Button freeButton;
    Button changeRoleButton;

    private View mProgressView;
    private View mCreateFormView;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_change_resource, container, false);
        getDialog().setTitle(R.string.title_fragment_change_resource);

        resource = (Resource)getArguments().getSerializable("resource");

        mCreateFormView = v.findViewById(R.id.intervention_scroll);
        mProgressView = v.findViewById(R.id.create_progress);

        changeRoleButton = (Button) v.findViewById(R.id.buton_change_role);
        changeRoleButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                changeResourceRole(role);
            }
        });

        //init fields
        validateButton = (Button) v.findViewById(R.id.validateButton);
        validateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                validateResource();
            }
        });
        freeButton = (Button) v.findViewById(R.id.freeButton);

        //init state spinner
        stateSpinner = (Spinner) v.findViewById(R.id.stateSpinner);
        stateSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String roleName = stateSpinner.getSelectedItem().toString();
                switch (roleName) {
                    case "Humain":
                        role = ResourceRole.people;
                        break;
                    case "Feu":
                        role = ResourceRole.fire;
                        break;
                    case "Eau":
                        role = ResourceRole.water;
                        break;
                    case "Risque":
                        role = ResourceRole.risks;
                        break;
                    case "Commande":
                        role = ResourceRole.commands;
                        break;
                    default:
                        role = ResourceRole.otherwise;
                        break;
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });

        //Selection of registered claims codes in the database and the list of identifiers of resourceType
        //showProgress(true);


        return v;
    }

    public void validateResource(){
        resource.setState(State.active);
        ((AgentInterventionActivity)getActivity()).resourceUpdated(resource);
        dismiss();
    }

    public void changeResourceRole(ResourceRole role){
        resource.setResourceRole(role);
        ((AgentInterventionActivity)getActivity()).resourceUpdated(resource);
        dismiss();
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
