#include "bits/stdc++.h"
#include "solver.h"
#define convertCoords(pt) pair<int,pair<int,pii>>(pt.box,pair<int,pii>(pt.x,pii(pt.y,pt.z)))
using namespace std;
vector<Uld> ULDList(6);
vector<Box>dat(400);

double weightz = 0.2;
double power_fac = 3.75;

const int LevelXYBoundWeight  =10;
void final_execution() {
    #ifndef GENETIC
    #ifdef OLD_SORTER
    Sorter Vol_Ht;
    Vol_Ht.val = [](Box a,Box b){
        if(b.isPriority and (not a.isPriority))return false;
        if(a.isPriority and (not b.isPriority))return true;
        // if(a.isPriority and b.isPriority)return false;
        if(a.cost!=b.cost)return a.cost>b.cost;
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h;
    };
    Sorter VolCost_Ht;
    VolCost_Ht.val = [](Box a,Box b){
        if(b.isPriority and (not a.isPriority))return false;
        if(a.isPriority and (not b.isPriority))return true;
        int vol = a.l*a.b*a.h, vol2 = b.l*b.b*b.h;
        return a.cost*vol2 > b.cost*vol;
    };
    Sorter Ht_Vol;
    Ht_Vol.val = [](Box a,Box b){
        if(a.h==b.h)return a.l*a.b*a.h > b.l*b.b*b.h;
        return a.h>b.h;
    };

    Sorter Area_Ht;  // Sort by base area, then by height
    Area_Ht.val = [](Box a, Box b) {
        if (a.l * a.b == b.l * b.b) return a.h > b.h;
        return a.l * a.b > b.l * b.b;
    };
    #endif
    Sorter Vol_Ht;
    Vol_Ht.val = [](vector<Box>& data){
        sort(data.begin(), data.end(), [](Box a,Box b){
            if(b.isPriority and (not a.isPriority))return false;
            if(a.isPriority and (not b.isPriority))return true;
            // if(a.isPriority and b.isPriority)return false;
            if(a.cost!=b.cost)return a.cost>b.cost;
            if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
            return a.l*a.b*a.h > b.l*b.b*b.h;
        });
    };
    Sorter Ashish_Ht;
    Ashish_Ht.val = [](vector<Box>& data){
        vector<int> mark(data.size()+5);
        for(int i =0; i != data.size(); i++){
            for(int j = 0; j != data.size(); j++){
                if(data[i].isPriority || data[j].isPriority)continue;
                if(data[i].cost <= data[j].cost)continue;
                vector<int> perm_i = {data[i].l, data[i].b, data[i].h}; sort(perm_i.begin(), perm_i.end());
                vector<int> perm_j = {data[j].l, data[j].b, data[j].h}; sort(perm_j.begin(), perm_j.end());
                if(perm_i[0] <= perm_j[0] && perm_i[1] <= perm_j[1] && perm_i[2] <= perm_j[2]){
                    if(!(perm_i[0] == perm_j[0] && perm_i[1] == perm_j[1] && perm_i[2] == perm_j[2]) && data[i].cost == data[j].cost){ // Ensure they don't correspond to the samme box either
                        mark[data[j].ID]++;
                        break;
                    }
                }
            }
        }
        
        sort(data.begin(), data.end(), [&](Box a,Box b){
            if(b.isPriority and (not a.isPriority))return false;
            if(a.isPriority and (not b.isPriority))return true;
            // if(mark[a.ID] != mark[b.ID] && abs(mark[a.ID] - mark[b.ID]) > 2) return mark[a.ID] < mark[b.ID];
            if(mark[a.ID] != mark[b.ID]) return mark[a.ID] < mark[b.ID];

            // if(a.isPriority and b.isPriority)return false;
            int vol1 = a.l*a.b*a.h, vol2 = b.l*b.b*b.h;
            // double power_fac = 1.0;
            double fac1 = pow(a.cost, power_fac)/(vol1*1.0), fac2 = pow(b.cost, power_fac)/(vol2*1.0);
            // return fac1 > fac2;
            // return a.cost > b.cost;

            if(a.cost!=b.cost)return fac1 > fac2;
            

            if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
            return a.l*a.b*a.h > b.l*b.b*b.h;
        });
    };

    
    //merits
    Merit MinVol;//redundant
    MinVol.val = [](coords c, Box b,Solver* s){
        return 1;
    };

    Merit minXYBound;  // Check bounds in XY plane
    minXYBound.val = [](coords c, Box box, Solver* s) {
        int ret = 0;
        int b = c.box;
        if (c.x + box.l > s->ULDl[b].maxBound.x) 
            ret += s->ULDl[b].maxBound.x - (c.x + box.l);
        if (c.y + box.b > s->ULDl[b].maxBound.y) 
            ret += s->ULDl[b].maxBound.y - (c.y + box.b);
        return ret;
    };

    Merit levelXYBound;  // Weighted bounds check
    levelXYBound.val = [](coords c, Box box, Solver* s) {
        int ret = 0;
        int b = c.box;
        if (c.x + box.l > s->ULDl[b].maxBound.x) 
            ret += (s->ULDl[b].maxBound.x - (c.x + box.l)) * LevelXYBoundWeight;
        else 
            ret += -(s->ULDl[b].maxBound.x - (c.x + box.l));
        if (c.y + box.b > s->ULDl[b].maxBound.y) 
            ret += (s->ULDl[b].maxBound.y - (c.y + box.b)) * LevelXYBoundWeight;
        else 
            ret += -(s->ULDl[b].maxBound.y - (c.y + box.b));
        return ret;
    };

    Merit Residue;  // Residual merit function
    Residue.val = residueFunc;

    // Initialize ULD dimensions and properties
    ULDList[0].dim.l = 224; ULDList[0].dim.b = 318; ULDList[0].dim.h = 162;
    ULDList[0].weight = 100;
    ULDList[0].maxBound = {0, 0, 0};

    // Read ULD input from file
    freopen("ULD.in", "r", stdin);
    for (int i = 0; i < ULDList.size(); ++i) {
        ULDList[i].weight = 0;
        cin >> ULDList[i].dim.l >> ULDList[i].dim.b >> ULDList[i].dim.h >> ULDList[i].maxWt;
        ULDList[i].maxBound = {0, 0, 0};
        ULDList[i].com = {ULDList[i].dim.l / 2, ULDList[i].dim.b / 2, ULDList[i].dim.h / 2};
    }

    // Read package data from file
    freopen("package.in", "r", stdin);
    for (int i = 0; i < dat.size(); ++i) {
        char c;
        cin >> c >> c >> dat[i].ID >> dat[i].l >> dat[i].b >> dat[i].h >> dat[i].weight;
        string s;
        cin >> s;
        dat[i].isPriority = (s == "Priority");
        cin >> dat[i].cost;
    }

    // Solve the packing problem
    // Solver s(Vol_Ht, Residue, dat, ULDList);
    
    // s.solve();
    // ScoredSolver s(Vol_Ht, Residue, dat, ULDList, 100);
    // s.solve();
    int _iter = 10;
    int CountPackages = 0, Cost = INF;
    double weightz_min = 0.2, power_fac_min = 3.75;
    // vector<Box> original_dat = dat; 
    // Output results
    // freopen("result.csv", "w", stdout);

    Sorter Final_Ht = Ashish_Ht;
    
    // for(weightz = 0.0; weightz <= 1.0; weightz += 0.01){
    //     for(power_fac= 3.0; power_fac <= 4.0; power_fac += 0.05){
    //         Solver s(Final_Ht, Residue, dat, ULDList);
    //         s.solve();
    //         // cout << "Weightz: " << power_fac << " gave me a cost of " << -s.cost() << endl;
    //         // cout.flush();
    //         // freopen("30998_result.csv", "w", stdout);
    //         if(-s.cost() <= Cost){
    //             cout << "Weightz: " << weightz << " Powerfac: " << power_fac << " gave me a cost of " << -s.cost() << endl;
    //             Cost = -s.cost();
    //             weightz_min = weightz;
    //             power_fac_min = power_fac;
    //         }
    //     }
    // }
    power_fac = power_fac_min;
    weightz = weightz_min;
    // ScoredSolver s(Final_Ht, Residue, dat, ULDList, 1000);
    int i;
    vector<int>visited(7);
    int TOTAL=0;
    vector<vector<Box>>dataa(6);
    dataa[0]=dat;
    for(int i=0;i<6;i++)
    {
        // cout << dat.size();
        ScoredSolver s(Final_Ht, Residue, dataa[i], ULDList, 0);
        s.solve();
        // cout.flush();
        int usedVol = 0, ULDVol =0;
        double maxef=0;int pack=-1;
        for(int j=0;j<6;j++)
        {
            // cout << j << " ";
            int vol =0;
            for(int box:s.ULDPackages[j])
            vol+=s.data[box].l*s.data[box].b*s.data[box].h;
            double p=1.0*vol/s.ULDl[j].dim.l/s.ULDl[j].dim.b/s.ULDl[j].dim.h;
            cout << p << "\n";
            if(maxef<p && !visited[j])
            {
                maxef=p;
                pack=j;
            }
        }
        visited[pack]=1;
        vector<Box>data1;
        vector<Uld>ULD1;
        ULDList[pack].maxWt=-1;
        for(auto x:s.ULDPackages[pack])
        data1.push_back(s.data[x]);
        vector<Uld>U;
        for(auto x:ULDList)
        {
            if(visited[x.ID-1])
            continue;
            U.push_back(x);
        }
        ULDList=U;
        cout << U.size();
        for(auto x:s.ULDPackages[pack])
        TOTAL+=s.data[x].cost;
        // cout << s.ULDPackages[pack].size() << " ";
        vector<Box>data2;
        int p=data1.size();
        // cout << p << "\n";
        for(int j=0;j<dat.size();j++)
        {
            int flag=0;
            for(int k=0;k<p;k++)
            {
                if(dat[j].b==data1[k].b && dat[j].l==data1[k].l && dat[j].h==data1[k].h && dat[j].weight==data1[k].weight)
                flag=1;
            }
            if(flag)continue;
            data2.push_back(dat[j]);
        }
        // cout << data2.size();
        dataa[i+1]=data2;
    }
    cout << TOTAL;
    // cout << "Writing minimum cost of " << -s.cost() << " for powerfac = " << power_fac << endl;
    // stringstream file_name;
    // file_name << "result_" << -s.cost() << ".csv";
    // s.writeToFile("new_result.csv");
    // weightz = weightz_min;
    // ScoredSolver s(Final_Ht, Residue, dat, ULDList, 1000);
    // s.solve();
    // cout << "Writing minimum cost of " << -s.cost() << " for weightz = " << weightz << endl;
    // s.writeToFile("new_result.csv");
    return;

    // FILE* file = freopen("result.csv", "w", stdout);
    // set<int> ULDPackages;
    // for (int i = 0; i < dat.size(); ++i) {
    //     if (s.placement[i].first.x != -1) {
    //         if (s.data[i].isPriority)
    //             ULDPackages.insert(s.placement[i].first.box);
    //         CountPackages++;
    //     }
    // }
    // cout << Cost << "," << CountPackages << "," << ULDPackages.size() << "\n";

    // for (int i = 0; i < dat.size(); ++i) {
    //     if (s.placement[i].first.x == -1) {
    //         cout << "P-" << s.data[i].ID << ",NONE,-1,-1,-1,-1,-1,-1\n";
    //     } else {
    //         cout << "P-" << s.data[i].ID << "," << "U" << s.placement[i].first.box + 1 << "," 
    //             << s.placement[i].first.x << "," << s.placement[i].first.y << "," 
    //             << s.placement[i].first.z << "," 
    //             << s.placement[i].second.l + s.placement[i].first.x << "," 
    //             << s.placement[i].second.b + s.placement[i].first.y << "," 
    //             << s.placement[i].second.h + s.placement[i].first.z << "\n";
    //     }
    // }
    // cout.flush();
    // fclose(file);
    #endif

    #ifdef GENETIC
    vector<Packet> pc;
    ParsePackets("packageNormal.txt", pc);
    vector<struct ULD> u;
    ParseULDs("ULDNormal.txt", u);

    Genetic gen(u, pc);
    gen.Execute();
    #endif
}

signed main(){
    ios_base::sync_with_stdio(0); cin.tie(0);cout.tie();
    final_execution();
}
