// #include "bits/stdc++.h"
#include "bits/stdc++.h"
#include "solver.h"
// #include "genetic.cpp"
#define int long long
#define For(i, n) for (int i = 0; i < n; i++)
#define FOR(k, i, n) for (int i = k; i < n; i++)
#define vi vector<int>
#define max(a, b) (a > b ? a : b)
#define maxP(a, b) (a.first > b.first ? a : b)
#define min(a, b) (a < b ? a : b)
#define INF 10000000000000000
#define pii pair<int, int>
#define PRIORITY_MISS_COST 1000000
#define NEW_ULD_PRIORITY_COST 5000
#define RESIDUE_THRESHOLD 0
#define convertCoords(pt) pair<int, pair<int, pii>>(pt.box, pair<int, pii>(pt.x, pii(pt.y, pt.z)))
using namespace std;

vector<Uld> ULDList(6);
vector<Box> dat(400);

#define WEIGHTZ_MIN 0.0
#define WEIGHTZ_MAX 0.5 
#define WEIGHTZ_INCREMENT 0.05

#define POWER_FAC_MIN 3
#define POWER_FAC_MAX 4
#define POWER_FAC_INCREMENT 0.05

double weightz = 0.1;
double power_fac = 3.7;

const int LevelXYBoundWeight = 10;
void final_execution()
{
#ifndef GENETIC
#ifdef OLD_SORTER
    Sorter Vol_Ht;
    Vol_Ht.val = [](Box a, Box b)
    {
        if (b.isPriority and (not a.isPriority))
            return false;
        if (a.isPriority and (not b.isPriority))
            return true;
        // if(a.isPriority and b.isPriority)return false;
        if (a.cost != b.cost)
            return a.cost > b.cost;
        if (a.l * a.b * a.h == b.l * b.b * b.h)
            return min(a.h, min(a.b, a.l)) < min(b.h, min(b.b, b.l));
        return a.l * a.b * a.h > b.l * b.b * b.h;
    };
    Sorter VolCost_Ht;
    VolCost_Ht.val = [](Box a, Box b)
    {
        if (b.isPriority and (not a.isPriority))
            return false;
        if (a.isPriority and (not b.isPriority))
            return true;
        int vol = a.l * a.b * a.h, vol2 = b.l * b.b * b.h;
        return a.cost * vol2 > b.cost * vol;
    };
    Sorter Ht_Vol;
    Ht_Vol.val = [](Box a, Box b)
    {
        if (a.h == b.h)
            return a.l * a.b * a.h > b.l * b.b * b.h;
        return a.h > b.h;
    };

    Sorter Area_Ht; // Sort by base area, then by height
    Area_Ht.val = [](Box a, Box b)
    {
        if (a.l * a.b == b.l * b.b)
            return a.h > b.h;
        return a.l * a.b > b.l * b.b;
    };
#endif
    Sorter Vol_Ht;
    Vol_Ht.val = [](vector<Box> &data)
    {
        sort(data.begin(), data.end(), [](Box a, Box b)
             {
            if(b.isPriority and (not a.isPriority))return false;
            if(a.isPriority and (not b.isPriority))return true;
            if(a.cost!=b.cost)return a.cost>b.cost;
            if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
            return a.l*a.b*a.h > b.l*b.b*b.h; });
    };
    /**
     * @brief This heuristic sorts the boxes,first by priority, then by marked, then by our suggested parameterized heuristic
     *      where mark denotes an object for which is a combination of cost, volume and height.
     */
    Sorter Marks_Cost_Vol_Ht;
    Marks_Cost_Vol_Ht.val = [](vector<Box> &data)
    {
        vector<int> mark(data.size() + 5);
        for (int i = 0; i != data.size(); i++)
        {
            for (int j = 0; j != data.size(); j++)
            {
                if (data[i].isPriority || data[j].isPriority)
                    continue;
                if (data[i].cost <= data[j].cost)
                    continue;
                vector<int> perm_i = {data[i].l, data[i].b, data[i].h};
                sort(perm_i.begin(), perm_i.end());
                vector<int> perm_j = {data[j].l, data[j].b, data[j].h};
                sort(perm_j.begin(), perm_j.end());
                if (perm_i[0] <= perm_j[0] && perm_i[1] <= perm_j[1] && perm_i[2] <= perm_j[2])
                {
                    if (!(perm_i[0] == perm_j[0] && perm_i[1] == perm_j[1] && perm_i[2] == perm_j[2]) && data[i].cost == data[j].cost)
                    { 
                        // Ensure they don't correspond to the same box either
                        mark[data[j].ID]++;
                        break;
                    }
                }
            }
        }

        sort(data.begin(), data.end(), [&](Box a, Box b)
             {
            if(b.isPriority and (not a.isPriority)){
                return false;
            }
            if(a.isPriority and (not b.isPriority)){
                return true;
            }
            if(mark[a.ID] != mark[b.ID]){
                return mark[a.ID] < mark[b.ID];
            }

            int vol1 = a.l*a.b*a.h, vol2 = b.l*b.b*b.h;
            double fac1 = pow(a.cost, power_fac)/(vol1*1.0), fac2 = pow(b.cost, power_fac)/(vol2*1.0);
            if(a.cost!=b.cost){
                return fac1 > fac2;
            }
            if(a.l*a.b*a.h==b.l*b.b*b.h){
                return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
            }
            return a.l*a.b*a.h > b.l*b.b*b.h; });
    };

    // merits
    Merit MinVol; // redundant
    MinVol.val = [](coords c, Box b, Solver *s)
    {
        return 1;
    };

    Merit minXYBound; // Check bounds in XY plane
    minXYBound.val = [](coords c, Box box, Solver *s)
    {
        int ret = 0;
        int b = c.box;
        if (c.x + box.l > s->ULDl[b].maxBound.x)
            ret += s->ULDl[b].maxBound.x - (c.x + box.l);
        if (c.y + box.b > s->ULDl[b].maxBound.y)
            ret += s->ULDl[b].maxBound.y - (c.y + box.b);
        return ret;
    };

    Merit levelXYBound; // Weighted bounds check
    levelXYBound.val = [](coords c, Box box, Solver *s)
    {
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

    Merit Residue; // Residual merit function
    Residue.val = residueFunc;

    // Read ULD input from file
    freopen("ULD.in", "r", stdin);
    for (int i = 0; i < ULDList.size(); ++i)
    {
        ULDList[i].weight = 0;
        cin >> ULDList[i].dim.l >> ULDList[i].dim.b >> ULDList[i].dim.h >> ULDList[i].maxWt;
        ULDList[i].maxBound = {0, 0, 0};
        ULDList[i].com = {ULDList[i].dim.l / 2, ULDList[i].dim.b / 2, ULDList[i].dim.h / 2};
    }

    // Read package data from file
    freopen("package.in", "r", stdin);
    for (int i = 0; i < dat.size(); ++i)
    {
        char c;
        cin >> c >> c >> dat[i].ID >> dat[i].l >> dat[i].b >> dat[i].h >> dat[i].weight;
        string s;
        cin >> s;
        dat[i].isPriority = (s == "Priority");
        cin >> dat[i].cost;
    }

    // Solve the packing problem

    /*Related to score-based method*/
    // int _iter = 1000;

    int cost = INF;

    double weightz_min = 0.1, power_fac_min = 3.7;
    Sorter Final_Ht = Marks_Cost_Vol_Ht;

    auto start = chrono::system_clock::now();
    // for(weightz = 0.0; weightz <= 0.5; weightz += 0.05){
    //     for(power_fac= 3; power_fac <= 4; power_fac += 0.05){
    //         Solver s(Final_Ht, Residue, dat, ULDList);
    //         s.solve();
    //         if(-s.cost() <= cost){
    //             cout << "Weightz: " << weightz << " Powerfac: " << power_fac << " gave me a cost of " << -s.cost() << endl;
    //             cost = -s.cost();
    //             weightz_min = weightz;
    //             power_fac_min = power_fac;
    //         }
    //     }
    // }
    auto end = chrono::system_clock::now();

    chrono::duration<double> elapsed_seconds = end - start;
    time_t end_time = chrono::system_clock::to_time_t(end);
    cout << "Heuristic Found Best Solution: " << cost << endl;
    cout << "Finished tuning heuristic at " << ctime(&end_time)
                << "Elapsed time: " << elapsed_seconds.count() << "s"
                << endl;
    power_fac = power_fac_min;
    weightz = weightz_min;
    Sorter emptySorter;
    emptySorter.val = [](vector<Box> &data)
    {
        return;
    };
    // Solver s(Final_Ht, Residue, dat, ULDList);
    // s.solve();
    // cout << s.cost() << endl;
    ScoredSolver s(Final_Ht, Residue, dat, ULDList, 0);
    s.solve();
    cout << s.cost() << endl;
#endif
}

signed main()
{
    ios_base::sync_with_stdio(0);
    cin.tie(0);
    cout.tie();
    final_execution();
}
