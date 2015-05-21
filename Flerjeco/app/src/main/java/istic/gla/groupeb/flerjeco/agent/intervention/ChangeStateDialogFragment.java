package istic.gla.groupeb.flerjeco.agent.intervention;

import android.content.DialogInterface;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.Spinner;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupb.nivimoju.util.ResourceRole;
import istic.gla.groupb.nivimoju.util.State;
import istic.gla.groupeb.flerjeco.springRest.IResourceActivity;
import istic.gla.groupeb.flerjeco.springRest.UpdateResourceTask;

/**
 * Fragment used for change the state of a resource of firefighters
 * @see android.support.v4.app.DialogFragment
 */
public class ChangeStateDialogFragment extends DialogFragment implements IResourceActivity {
    private static final String TAG = ChangeStateDialogFragment.class.getSimpleName();

    Spinner stateSpinner;
    Resource resource;
    private ResourceRole role;
    private UpdateResourceTask updateResourceTask;

    //fields
    CheckBox validatecheckBox;
    Button freeButton;
    Button changeRoleButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View v = inflater.inflate(R.layout.fragment_change_resource, container, false);
        getDialog().setTitle(R.string.title_fragment_change_resource);

        resource = (Resource)getArguments().getSerializable("resource");

        validatecheckBox = (CheckBox) v.findViewById(R.id.validateCheckBox);
        if(resource.getResourceCategory() == ResourceCategory.dragabledata) {
            validatecheckBox.setVisibility(View.INVISIBLE);
        }
        if(State.active.equals(resource.getState())) {
            validatecheckBox.setChecked(true);
            validatecheckBox.setEnabled(false);
        }
        changeRoleButton = (Button) v.findViewById(R.id.buton_change_role);
        changeRoleButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(validatecheckBox != null) {
                    changeResourceRoleAndActivation(role);
                }
            }
        });

        //init fields

        freeButton = (Button) v.findViewById(R.id.freeButton);
        freeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                releaseResource();
            }
        });

        //init state spinner
        if (resource.getResourceCategory() == ResourceCategory.dragabledata) {
            stateSpinner = (Spinner) v.findViewById(R.id.stateSpinnerDraggable);
            v.findViewById(R.id.stateSpinnerVehicle).setVisibility(View.GONE);
        } else {
            stateSpinner = (Spinner) v.findViewById(R.id.stateSpinnerVehicle);
            v.findViewById(R.id.stateSpinnerDraggable).setVisibility(View.GONE);
        }

        int position = getStatePosition();
        if (position != -1) {
            stateSpinner.setSelection(position);
        }

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
        ((AgentInterventionActivity)getActivity()).updateResource(resource);
        dismiss();
    }

    public void changeResourceRoleAndActivation(ResourceRole role){
        if(validatecheckBox.isChecked()){
            validateResource();
        }
        resource.setResourceRole(role);
        ((AgentInterventionActivity)getActivity()).updateResource(resource);
        dismiss();
    }

    public void releaseResource() {
        updateResourceTask = new UpdateResourceTask(this);
        updateResourceTask.execute(
                "" + ((AgentInterventionActivity) getActivity()).getIntervention().getId(),
                "" + resource.getIdRes(),
                State.free.name());
    }

    public int getStatePosition(){
        String toCompare = "";
        switch (resource.getResourceRole()){
            case people:
                toCompare = "Humain";
                break;
            case fire:
                toCompare = "Feu";
                break;
            case water:
                toCompare = "Eau";
                break;
            case risks:
                toCompare = "Risque";
                break;
            case commands:
                toCompare = "Commande";
                break;
            default:
                break;
        }
        if (stateSpinner != null){
            for (int i = 0; i < stateSpinner.getAdapter().getCount(); i++) {
                String state = stateSpinner.getAdapter().getItem(i).toString();
                if (toCompare.equals(state)){
                    return i;
                }
            }
        }
        return -1;
    }

    @Override
    public void onPause() {
        super.onPause();
    }

    @Override
    public void onCancel(DialogInterface dialog) {
        super.onCancel(dialog);
        if(updateResourceTask != null) {
            updateResourceTask.cancel(true);
        }
    }

    @Override
    public void updateResources(Intervention intervention) {
        if(intervention != null) {
            dismiss();
        }
    }
}
