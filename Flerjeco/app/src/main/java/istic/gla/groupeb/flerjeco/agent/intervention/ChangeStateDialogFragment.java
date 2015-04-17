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
 * @see  istic.gla.groupeb.flerjeco.codis.intervention.OnTaskCompleted
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

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_change_resource, container, false);
        getDialog().setTitle(R.string.title_fragment_change_resource);

        resource = (Resource)getArguments().getSerializable("resource");

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

    @Override
    public void onPause() {
        super.onPause();

    }


}
