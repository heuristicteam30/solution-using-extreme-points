//#include "bits/stdc++.h"
#include "bits/stdc++.h"
#include "solver.h"
//#include "genetic.cpp"
#define int long long
#define For(i,n) for(int i=0; i<n;i++)
#define FOR(k,i,n) for(int i=k; i<n;i++)
#define vi vector<int>
#define max(a,b) (a>b?a:b)
#define maxP(a,b) (a.first>b.first?a:b)
#define min(a,b) (a<b?a:b)
#define INF 10000000000000000
#define pii pair<int,int>
#define NON_PRIORITY_COST 1000000
#define PRIORITY_ULD_COST 5000
#define RESIDUE_THRESHOLD 0
#define convertCoords(pt) pair<int,pair<int,pii>>(pt.box,pair<int,pii>(pt.x,pii(pt.y,pt.z)))
using namespace std;
vector<Uld> ULDList(6);
vector<Box>dat(400);
const int LevelXYBoundWeight  =10;
void final_execution() {
    #ifndef GENETIC
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

    // Define merit functions for different evaluation criteria
    Merit MinVol;  // Placeholder merit function
    MinVol.val = [](coords c, Box b, Solver* s) {
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
    // Solver s(Vol_Ht, Residue, dat, ULDList);
    // s.solve();
    ScoredSolver s(Vol_Ht, Residue, dat, ULDList, 100);
    s.solve();

    // Output results
    freopen("result.csv", "w", stdout);
    int CountPackages = 0, Cost = -s.cost();
    set<int> ULDPackages;

    for (int i = 0; i < dat.size(); ++i) {
        if (s.placement[i].first.x != -1) {
            if (s.data[i].isPriority) 
                ULDPackages.insert(s.placement[i].first.box);
            CountPackages++;
        }
    }
    cout << Cost << "," << CountPackages << "," << ULDPackages.size() << "\n";

    for (int i = 0; i < dat.size(); ++i) {
        if (s.placement[i].first.x == -1) {
            cout << s.data[i].ID << ",NONE,-1,-1,-1,-1,-1,-1\n";
        } else {
            cout << s.data[i].ID << "," << s.placement[i].first.box + 1 << "," 
                 << s.placement[i].first.x << "," << s.placement[i].first.y << "," 
                 << s.placement[i].first.z << "," 
                 << s.placement[i].second.l + s.placement[i].first.x << "," 
                 << s.placement[i].second.b + s.placement[i].first.y << "," 
                 << s.placement[i].second.h + s.placement[i].first.z << "\n";
        }
    }

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
