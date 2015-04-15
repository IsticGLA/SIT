package istic.gla.groupeb.flerjeco.adapter;

import android.app.Activity;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;

import java.util.List;

import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;

/**
 * Created by erwann on 09/04/15.
 */
public class IconViewAdapter extends ArrayAdapter<IconView> {

    Context context;
    int layoutResId;
    List<IconView> iconViewList;

    public IconViewAdapter(Context context, int layoutResId, List<IconView> iconViewList) {
        super(context, layoutResId, iconViewList);
        this.context = context;
        this.layoutResId = layoutResId;
        this.iconViewList = iconViewList;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent) {
        View row = convertView;
        IconHolder holder = null;

        if (row == null) {
            LayoutInflater inflater = ((Activity)context).getLayoutInflater();
            row = inflater.inflate(R.layout.list_row, parent, false);
            //row = LayoutInflater.from(context).inflate(R.layout.list_row, null);
            holder = new IconHolder();
            holder.iconView = (IconView)row.findViewById(R.id.icon_view);
            row.setTag(holder);

        } else {
            holder = (IconHolder)row.getTag();
        }

        IconView iconView = iconViewList.get(position);
        holder.iconView.setIcon(iconView.getIcon());
        return row;
    }

    static class IconHolder {
        IconView iconView;
    }
}
