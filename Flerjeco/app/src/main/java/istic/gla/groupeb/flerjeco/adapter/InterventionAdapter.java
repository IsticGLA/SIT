package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupeb.flerjeco.R;

/**
 * adapter for showing interventions on listview
 */
public class InterventionAdapter extends ArrayAdapter<Intervention> {
    private static final String TAG = InterventionAdapter.class.getSimpleName();

    /**
     * private viewholder for performances
     */
    private static class ViewHolder {
        TextView interventionName;
    }

    public InterventionAdapter(Context context, ArrayList<Intervention> list){
        super(context, 0, list);
    }

    @Override
    public View getView(int position, View view, ViewGroup viewGroup) {
        Intervention intervention = getItem(position);

        //check to see if the reused view is null or not, if is not null then reuse it
        ViewHolder viewHolder;
        if (view == null) {
            viewHolder = new ViewHolder();
            view = LayoutInflater.from(getContext()).inflate(R.layout.item_intervention_codis, null);
            viewHolder.interventionName = (TextView) view.findViewById(R.id.item_intervention_name);
            view.setTag(viewHolder);
        }else  {
            viewHolder = (ViewHolder) view.getTag();
        }
        viewHolder.interventionName.setText(intervention.getName());
        //this method must return the view corresponding to the data at the specified position.
        return view;
    }

    @Override
    public boolean isEmpty(){
        Log.d(TAG, "isempty : " + super.isEmpty());
        return super.isEmpty();
    }
}
