//
// Created by 研究用 on 2017/09/06.
//

#include "Plan.h"
#include "parameter.h"

#include <string>
#include <limits>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

using namespace std;

Plan::Plan(vector<vector<int> > grid_input, IJ start_input, IJ goal_input) {
    //install grid, start, goal
    grid=grid_input;
    start=start_input;
    goal=goal_input;

    int tate=grid.size();
    int yoko=grid[0].size();

    //evaluate h table
    h=grid; //this is just to resize h
    evaluate_h();

    //put start in the list
    GIJ start_G;
    start_G.set_GIJ(0,start.getI(),start.getJ());
    list.push_back(start_G);

    //initialize expand table
    count_expand=0;
    expand_table=grid;  //this is just to resize expand table
    for(int i=0;i<tate;i++){
        for(int j=0;j<yoko;j++) expand_table[i][j]=-1;
    }
    expand_table[list[0].getI()][list[0].getJ()]=count_expand;

    //initialize f-table
    f=grid; //this is just to resize f
    for(int i=0;i<tate;i++){
        for(int j=0;j<yoko;j++) f[i][j]=-1;
    }
    f[start.getI()][start.getJ()]=h[start.getI()][start.getJ()];

}

//////////////////Path planning////////////////
void Plan::evaluate_h() {
    for(int i=0;i<h.size();i++){
        for(int j=0;j<h[0].size();j++){
            int tate=abs(i-goal.getI());
            int yoko=abs(j-goal.getJ());
            h[i][j]=tate+yoko;
        }
    }
}

//solve the problem with A* algorithm
void Plan::proceed_Astar() {
    cout<<"solve the grid problem with A* algorithm"<<endl;
    bool condition;
    while(1){
//        show_list();
        //if it reaches at the goal, stop the loop
        condition=judge();
        if(condition==true) break;
        //if list is empty, the search fails
        if(list.empty()){
            cout<<"Fail!"<<endl;
            break;
        }
        expand_next_Astar();      //if it is not at the goal, expand the list
    }
}

void Plan::expand_next_Astar() {
    //choose the point with the least f-value [g_min,i,j]
    int f_min=INT_MAX;
    int g_min;
    int temp_id;
    for(int i=0;i<list.size();i++){
        int f_temp=list[i].getValue()+h[list[i].getI()][list[i].getJ()];
//list[i].show();
//cout<<"f-value:"<<f_temp<<endl;
        if(f_temp<f_min) {
            f_min=f_temp;
            g_min=list[i].getValue();
            temp_id=i;
        }
    }
//cout<<"expand this: f-value is "<<f_min<<", ";
//list[temp_id].show();

    //erase the point with the least f-value, and put it in checked list
    int i=list[temp_id].getI();
    int j=list[temp_id].getJ();
    list.erase(list.begin()+temp_id);
    GIJ temp;
    temp.set_GIJ(g_min,i,j);
    checked.push_back(temp);    //put the point to the checked list

    //expand from this point (without choosing the neighbor with the least f-value)
    int i_next,j_next;
    //case 1: up
    i_next=i-1;
    j_next=j;
    expand(g_min,i_next,j_next);

    //case 2:down
    i_next=i+1;
    j_next=j;
    expand(g_min,i_next,j_next);

    //case 3:right
    i_next=i;
    j_next=j+1;
    expand(g_min,i_next,j_next);

    //case 4:left
    i_next=i;
    j_next=j-1;
    expand(g_min,i_next,j_next);
}

//Expand the specified tile with incremented g-value (if possible)
void Plan::expand(int g_input, int i_input,int j_input) {
    GIJ temp;
    int tate=grid.size();
    int yoko=grid[0].size();
    bool condition=true;

    //check whether the specified tile is already in checked list
    for(int k=0;k<checked.size();k++){
        if(checked[k].getI()==i_input && checked[k].getJ()==j_input) condition=false;
    }
    //check whether the specified tile is already in list
    for(int k=0;k<list.size();k++){
        if(list[k].getI()==i_input && list[k].getJ()==j_input) condition=false;
    }
    //check whether the specified tile is in grid, or blocked tile
    if(0<=i_input && i_input<tate && 0<=j_input && j_input<yoko //condition 1:Is it inside the grid?
       && grid[i_input][j_input]==0 //condition 2:Is it unblocked tile?
       && condition==true)        //condition 3:Is it already checked or in the list?
    {   //if these conditions are valid, expand with incrementing g-value by 1
        temp.set_GIJ(g_input+1,i_input,j_input);
        list.push_back(temp);
        count_expand++;
//cout<<"count becomes "<<count_expand<<", and the expanded tile is ["<<i_input<<" "<<j_input<<"], put g-number:"<<g_input+1<<endl;
        expand_table[i_input][j_input]=count_expand;    //put expansion number in table
        f[i_input][j_input]=h[i_input][j_input]+g_input+1;  //put f-number
    }
}

//judge whether it reaches at the goal
bool Plan::judge() {
    for(int i=0;i<list.size();i++){
        if(list[i].getI()==goal.getI() && list[i].getJ()==goal.getJ()) return true;
    }
    return false;
}

void Plan::show_expand_table() {
    cout<<"Show expand table"<<endl;
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[i].size();j++) cout<<expand_table[i][j]<<" ";
        cout<<endl;
    }
}

//find path. Start from the goal to the starting point
void Plan::find_path() {
    //pick up the goal position [g,i,j]
    int g=number_step;
    int i=goal.getI();
    int j=goal.getJ();
    path.push_back(goal);   //put goal in the path

    //count down from g-value at the goal
    for(int n=number_step;n>=0;n--){
        //pick up from the checked tiles
        for(int m=checked.size();m>=0;m--){
            //check whether the specified tile is neighboring in the sense of g-value and [i,j]
            bool neighbor=false;
            if((checked[m].getI()==i-1 && checked[m].getJ()==j)
               || (checked[m].getI()==i+1 && checked[m].getJ()==j)
               || (checked[m].getI()==i && checked[m].getJ()==j+1)
               || (checked[m].getI()==i && checked[m].getJ()==j-1)) neighbor=true;
            //if it is neighbor, move to this tile [g,i,j] and put it in path
            if(checked[m].getValue()==g-1 && neighbor==true){
                path.push_back(checked[m]);
                g--;
                i=checked[m].getI();
                j=checked[m].getJ();
            }
        }
    }
    reverse(path.begin(),path.end());   //reversing path so that it becomes (start -> goal)
}

///////////functions for processing (path smoothing)///////////
void Plan::convert_path_ij_xy() {
    Path.resize(path.size());
    for(int i=0;i<path.size();i++){
        double x=path[i].getJ();
        double y=grid.size()-1-path[i].getI();
        XY temp;
        temp.setXY(x,y);
        Path[i]=temp;
//cout<<"[x="<<x<<", y="<<y<<"]"<<endl;
    }
    OriginalPath=PreviousPath=Path;
}

void Plan::smooth_path() {
    cout<<"smoothen path"<<endl;
    while(1){
        for(int i=1;i<path.size()-1;i++){   //exclude the initial and final points
            //minimizing (yi-xi)^2 (close to the original path), minimizing (yi+1+yi-1-2yi)^2 (smooth path)
            double path_i_i=PreviousPath[i].getX()+weight_data*(OriginalPath[i].getX()-PreviousPath[i].getX())
                            +weight_smooth*(PreviousPath[i+1].getX()+PreviousPath[i-1].getX()-2*PreviousPath[i].getX());
            double path_i_j=PreviousPath[i].getY()+weight_data*(OriginalPath[i].getY()-PreviousPath[i].getY())
                            +weight_smooth*(PreviousPath[i+1].getY()+PreviousPath[i-1].getY()-2*PreviousPath[i].getY());
            //improve (if it is harmful, remove it)
            if(2<=i && i<=path.size()-3){
                path_i_i+=0.5*weight_smooth*(2*PreviousPath[i+1].getX() + 2*PreviousPath[i-1].getX()
                                             -PreviousPath[i+2].getX() -PreviousPath[i-2].getX()
                                             - 2 * PreviousPath[i].getX());
                path_i_j+=0.5*weight_smooth*(2*PreviousPath[i+1].getY() + 2*PreviousPath[i-1].getY()
                                             -PreviousPath[i+2].getY() -PreviousPath[i-2].getY()
                                             - 2 * PreviousPath[i].getY());
            }
            Path[i].setXY(path_i_i,path_i_j);
//path[i].show();
        }

        if(judge_smoothness()==true) break;
        PreviousPath=Path;
    }
}

bool Plan::judge_smoothness() {
    double change=0;
    for(int i=0;i<path.size();i++) change+=Path[i].distance(PreviousPath[i]);

//    cout<<"change:"<<change<<endl;
    if(change<=tolerance_smoothing) return true;
    else return false;
}

void Plan::show_change_path() {
    cout<<"show change of path"<<endl;
    for(int i=0;i<path.size();i++){
        OriginalPath[i].show_nonewline();
        cout<<"->";
        Path[i].show_nonewline();
        cout<<endl;
    }
}

void Plan::write_change_path() {
    string filename="change_path.dat";
    ofstream fout;  //書き込み用のofstreamを宣言
    fout.open(filename,ios::out);

    cout<<"write results"<<endl;
    for(int i=0;i<path.size();i++) fout<< OriginalPath[i].getX()<<" "<<OriginalPath[i].getY()<<endl;
    fout<<endl<<endl;
    for(int i=0;i<path.size();i++) fout<< Path[i].getX()<<" "<<Path[i].getY()<<endl;

    fout.close();
}

void Plan::write_block() {
    string filename="block.dat";
    ofstream fout;  //書き込み用のofstreamを宣言
    fout.open(filename,ios::out);

    cout<<"write results"<<endl;
    for(int i=0;i<grid.size();i++) {
        for(int j=0;j<grid[0].size();j++){
            if(grid[i][j]==1){
                //draw circle with radius=0.5
                double x_center=j;
                double y_center=grid.size()-1-i;
                double theta_max=2.0*PI;
                double delta_theta=0.3;
                int n_max=(int)(theta_max/delta_theta);
                for(int n=0;n<=n_max;n++){
                    double theta=delta_theta*n;
                    double x=0.5*cos(theta)+x_center;
                    double y=0.5*sin(theta)+y_center;
                    fout<<x<<" "<<y<<endl;
                }
            }
            fout<<endl<<endl;
        }
    }
    fout.close();
}




//////////////////Interface////////////////
void Plan::show_grid() {
    cout<<"Show grid"<<endl;
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[i].size();j++) cout<<grid[i][j]<<" ";
        cout<<endl;
    }
}

void Plan::show_g() {
    cout<<"Show g-value"<<endl;
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[i].size();j++) {
            if(grid[i][j]==0) cout<<f[i][j]-h[i][j]<<" ";
            else cout<<-1<<" ";
        }
        cout<<endl;
    }
}

void Plan::show_h() {
    cout<<"Show h"<<endl;
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[i].size();j++) cout<<h[i][j]<<" ";
        cout<<endl;
    }
}

void Plan::show_f() {
    cout<<"Show f"<<endl;
    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[i].size();j++) cout<<f[i][j]<<" ";
        cout<<endl;
    }
}

//print the resultant number of steps
void Plan::show_number_step() {
    cout<<"Resultant number of steps:";
    for(int i=list.size()-1;i>=0;i--){
        if(list[i].getI()==goal.getI() && list[i].getJ()==goal.getJ()){
            number_step=list[i].getValue();
            cout<<number_step<<endl;
        }
    }
    cout<<"goal position:["<<goal.getI()<<", "<<goal.getJ()<<"]"<<endl;
}

void Plan::show_list() {
    cout<<"Show list"<<endl;
    for(int i=0;i<list.size();i++) list[i].show();
    cout<<endl;
}

void Plan::show_checked() {
    cout<<"Show checked list"<<endl;
    for(int i=0;i<checked.size();i++) checked[i].show();
}

void Plan::show_path() {
    cout<<"Show path"<<endl;
    for(int i=0;i<path.size();i++) path[i].show();
}

const vector<XY> &Plan::getPath() const {
    return Path;
}



