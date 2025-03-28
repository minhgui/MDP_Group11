package com.example.mdp_group_11;

import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import androidx.fragment.app.FragmentPagerAdapter;

import java.util.ArrayList;

public class SectionsPagerAdapter extends FragmentPagerAdapter {
    private final ArrayList<Fragment> fragList = new ArrayList<>();
    private final ArrayList<String> fragTitle = new ArrayList<>();


    public SectionsPagerAdapter(FragmentManager fm, int behaviour) {
        super(fm,behaviour);
    }

    @Override
    public Fragment getItem(int position) {
        return fragList.get(position);
    }

    @Nullable
    @Override
    public CharSequence getPageTitle(int position) {
        return fragTitle.get(position);
    }

    @Override
    public int getCount() {
        return fragList.size();
    }

    public void addFragment(Fragment fm, String title){
        fragList.add(fm);
        fragTitle.add(title);
    }
}
