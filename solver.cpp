#include "solver.h"
#define int long long
#define For(i, n) for (int i = 0; i < n; i++)
#define FOR(k, i, n) for (int i = k; i < n; i++)
#define vi vector<int>
#define max(a, b) (a > b ? a : b)
#define maxP(a, b) (a.first > b.first ? a : b)
#define min(a, b) (a < b ? a : b)
#define INF 10000000000000000
#define pii pair<int, int>
#define NON_PRIORITY_COST 1000000
#define PRIORITY_ULD_COST 5000
#define RESIDUE_THRESHOLD 0
#define convertCoords(pt) pair<int, pair<int, pii>>(pt.box, pair<int, pii>(pt.x, pii(pt.y, pt.z)))

using namespace std;

extern double weightz, power_fac;

bool check(const pair<pair<int, int>, pair<int, int>> &a, const pair<pair<int, int>, pair<int, int>> &b)
{ // custom comparator to sort
    int p = a.second.second;
    int q = b.second.second;
    return (p > q); // returns box with higher height
}
Solver::Solver(Sorter sorter_, Merit merit_, vector<Box> boxes, vector<Uld> ULD_) // constructor
{
    this->merit = merit_;
    this->sorter = sorter_;
    this->data = boxes;
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;

    // initialization
    this->placement.assign(boxes.size(), pair<coords, Box>(def, def_));
    ULDl = ULD_;
    ULDPackages.assign(ULDl.size(), set<int>());
    surfaces.assign(ULDl.size(), set<pair<int, pair<pair<int, int>, pair<int, int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);

    // comparator function definitoins for additional EP creation
    compare_x = [this](const int &a, const int &b)
    { return placement[a].first.x + placement[a].second.l > placement[b].first.x + placement[b].second.l; };
    compare_y = [this](const int &a, const int &b)
    { return placement[a].first.y + placement[a].second.b > placement[b].first.y + placement[b].second.b; };
    compare_z = [this](const int &a, const int &b)
    { return placement[a].first.z + placement[a].second.h > placement[b].first.z + placement[b].second.h; };

    ULD_sorted_x.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_x));
    ULD_sorted_y.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_y));
    ULD_sorted_z.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_z));

    // sort blocking pkts in increasing order of base face coordinate
    compare_x_base = [this](const int &a, const int &b)
    { return placement[a].first.x < placement[b].first.x; };
    compare_y_base = [this](const int &a, const int &b)
    { return placement[a].first.y < placement[b].first.y; };
    compare_z_base = [this](const int &a, const int &b)
    { return placement[a].first.z < placement[b].first.z; };

    ULD_blocking_boxes_x = set<int, function<bool(const int &, const int &)>>(compare_x_base);
    ULD_blocking_boxes_y = set<int, function<bool(const int &, const int &)>>(compare_y_base);
    ULD_blocking_boxes_z = set<int, function<bool(const int &, const int &)>>(compare_z_base);
}

bool Solver::writeToFile(string filename)
{
    set<int> ULDPackages;
    int pkgCount = 0;
    for (int i = 0; i < data.size(); ++i)
    {
        if (placement[i].first.x != -1)
        {
            if (data[i].isPriority)
                ULDPackages.insert(placement[i].first.box);
            pkgCount++;
        }
    }
    ofstream file;
    file.open(filename);
    if (!file.is_open())
        return false;
    file << -this->cost() << "," << pkgCount << "," << ULDPackages.size() << "\n";
    for (int i = 0; i < data.size(); ++i)
    {
        if (placement[i].first.x == -1)
        {
            file << "P-" << data[i].ID << ",NONE,-1,-1,-1,-1,-1,-1\n";
        }
        else
        {
            file << "P-" << data[i].ID << "," << "U" << placement[i].first.box + 1 << ","
                 << placement[i].first.x << "," << placement[i].first.y << ","
                 << placement[i].first.z << ","
                 << placement[i].second.l + placement[i].first.x << ","
                 << placement[i].second.b + placement[i].first.y << ","
                 << placement[i].second.h + placement[i].first.z << "\n";
        }
    }
    file.close();
    return true;
}

int Solver::cost()
{
    int totalcost = 0;          // to store total cost of solution
    set<int> priorityShipments; // to store ULDs containing priority packages
    for (int i = 0; i < data.size(); i++)
    {
        if (placement[i].first.x == -1) // if the package is not placed in any ULD
        {
            totalcost += data[i].cost; // add the cost of economy package to the toal
        }
        else
        {
            if (data[i].isPriority)
                priorityShipments.insert(placement[i].first.box); // insert the ULD-ID into the set "priorityShipments"
        }
    }
    totalcost += priorityShipments.size() * PRIORITY_ULD_COST; //  add the cost of priority packages in different ULDs to the total cost
    return totalcost;
}

bool Solver::checkCollision(coords e, Box b)
{
    // checks collision of prespective packages with all other packages of same ULD along with weight constraint of the ULD
    // if (e.x + b.l > ULDl[e.box].dim.l or e.y + b.b > ULDl[e.box].dim.b or e.z + b.h >         ULDl[e.box].dim.h or ULDl[e.box].weight + b.weight > ULDl[e.box].maxWt)
    if (e.x + b.l >= ULDl[e.box].dim.l or e.y + b.b >= ULDl[e.box].dim.b or e.z + b.h >= ULDl[e.box].dim.h or ULDl[e.box].weight + b.weight > ULDl[e.box].maxWt)
        return true;

    // check collision of the given packages with all other packages of same ULD
    for (auto i : ULDPackages[e.box])
    {
        auto x = placement[i];
        if ((x.first.x < b.l + e.x and x.first.y < b.b + e.y and x.first.z < b.h + e.z and e.x < x.first.x + x.second.l and e.y < x.first.y + x.second.b and e.z < x.first.z + x.second.h))
        {
            return true;
        }
    }
    return false;
}

void Solver::solve()
{
// int c=0;
// cout << weightz << endl;
#ifndef GENETIC
#ifdef OLD_SORT
    sort(data.begin(), data.end(), this->sorter.val);
#endif
#ifndef OLD_SORT
    this->sorter.val(data);
#endif
#endif

    for(int i=0;i<ULDl.size();i++) ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
    for(int i=0;i<data.size();i++)
    {
        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h;
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if (checkCollision(e, p)) // checks for overlapping if a package is placed on this ULD
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        if (data[i].isPriority)
            ULDHasPriority[best.second.first.box] = true;
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        // addEP(i);
        addEP2(i);
        update(i);
    }
    
}

void Solver::update(int i)
{
    int b = placement[i].first.box;
    updateMaxBound(i);
    updateResidue(i);
    ULDl[b].com.x += (placement[i].first.x + placement[i].second.l / 2) * data[i].weight;
    ULDl[b].com.y += (placement[i].first.y + placement[i].second.b / 2) * data[i].weight;
    ULDl[b].com.z += (placement[i].first.z + placement[i].second.h / 2) * data[i].weight;
    ULDl[b].weight += data[i].weight;
    surfaces[b].insert(make_pair(placement[i].first.z + placement[i].second.h, make_pair(make_pair(placement[i].first.x, placement[i].first.y), make_pair(placement[i].first.x + placement[i].second.l, placement[i].first.y + placement[i].second.b))));
}

vector<coords> Solver::getCOM()
{
    vector<coords> r(ULDl.size());
    for (int i = 0; i < r.size(); i++)
    {
        if (ULDl[i].weight == 0)
        {
            r[i].x = ULDl[i].dim.l / 2;
            r[i].y = ULDl[i].dim.b / 2;
            r[i].z = ULDl[i].dim.h / 2;
            continue;
        }
        r[i].x = ULDl[i].com.x / ULDl[i].weight;
        r[i].y = ULDl[i].com.y / ULDl[i].weight;
        r[i].z = ULDl[i].com.z / ULDl[i].weight;
    }
    return r;
}

pair<int, pair<int, int>> Solver::getResidueSpace(coords src)
{
    pair<int, pair<int, int>> ret(beamprojectXPos(src).x - src.x, pair<int, int>(beamprojectYPos(src).y - src.y, beamprojectZPos(src).z - src.z));
    return ret;
}

void Solver::initialise(coords &X, coords &ob, int x, int y, int z)
{
    X.x = ob.x + x;
    X.y = ob.y + y;
    X.z = ob.z + z;
    X.box = ob.box;
}

coords Solver::beamprojectZNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 1, 0);
    initialise(r, ob1, 1, 1, 0);
    coords p1 = rayProjectZNeg(p);
    coords q1 = rayProjectZNeg(ob1);
    coords r1 = rayProjectZNeg(q);
    coords s1 = rayProjectZNeg(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = ob1.y;
    ans.z = min(min(min(p1.z, q1.z), r1.z), s1.z);
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectYNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 1, 0, 1);
    coords p1 = rayProjectYNeg(p);
    coords q1 = rayProjectYNeg(ob1);
    coords r1 = rayProjectYNeg(q);
    coords s1 = rayProjectYNeg(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = min(min(min(p1.y, q1.y), r1.y), s1.y);
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectXNeg(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 0, 1, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 0, 1, 1);
    coords p1 = rayProjectXNeg(p);
    coords q1 = rayProjectXNeg(ob1);
    coords r1 = rayProjectXNeg(q);
    coords s1 = rayProjectXNeg(r);
    coords ans;
    ans.x = min(min(min(p1.x, q1.x), r1.x), s1.x);
    ans.y = ob1.y;
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectZPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 1, 0);
    initialise(r, ob1, 1, 1, 0);
    coords p1 = rayProjectZPos(p);
    coords q1 = rayProjectZPos(ob1);
    coords r1 = rayProjectZPos(q);
    coords s1 = rayProjectZPos(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = ob1.y;
    ans.z = min(min(min(p1.z, q1.z), r1.z), s1.z);
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectYPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 1, 0, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 1, 0, 1);
    coords p1 = rayProjectYPos(p);
    coords q1 = rayProjectYPos(ob1);
    coords r1 = rayProjectYPos(q);
    coords s1 = rayProjectYPos(r);
    coords ans;
    ans.x = ob1.x;
    ans.y = min(min(min(p1.y, q1.y), r1.y), s1.y);
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}

coords Solver::beamprojectXPos(coords ob1)
{
    coords p, q, r;
    initialise(p, ob1, 0, 1, 0);
    initialise(q, ob1, 0, 0, 1);
    initialise(r, ob1, 0, 1, 1);
    coords p1 = rayProjectXPos(p);
    coords q1 = rayProjectXPos(ob1);
    coords r1 = rayProjectXPos(q);
    coords s1 = rayProjectXPos(r);
    coords ans;
    ans.x = min(min(min(p1.x, q1.x), r1.x), s1.x);
    ans.y = ob1.y;
    ans.z = ob1.z;
    ans.box = ob1.box;
    return ans;
}
bool Solver::check_collision_2D(int x1_min, int x1_max, int y1_min, int y1_max, int x2_min, int x2_max, int y2_min, int y2_max)
{
    bool is_x2_min_inside = (x2_min >= x1_min) && (x2_min <= x1_max);
    bool is_x2_max_inside = (x2_max > x1_min) && (x2_max <= x1_max);
    bool is_y2_min_inside = (y2_min >= y1_min) && (y2_min <= y1_max);
    bool is_y2_max_inside = (y2_max > y1_min) && (y2_max <= y1_max);
    return (is_x2_min_inside || is_x2_max_inside) && (is_y2_min_inside || is_y2_max_inside);
}
void Solver::projection_neg_z_advanced(vector<coords> &advanced_eps, coords start, int self_pid)
{

    bool hit_bottom = true;
    for (auto i : ULD_sorted_z[start.box])
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = start.y;
        potential_ep.z = placement[i].first.z + placement[i].second.h;

        if (i == self_pid)
            continue;
        auto pkt = placement[i];
        if (pkt.first.x == -1)
            continue;
        if (pkt.first.x + pkt.second.l <= start.x)
            continue;
        if (pkt.first.y + pkt.second.b <= start.y)
            continue;
        if (pkt.first.z >= start.z)
            continue;

        // case 0: ray hits the object (no more projections)
        if ((start.x >= pkt.first.x) && (start.x < pkt.first.x + pkt.second.l) &&
            (start.y >= pkt.first.y) && (start.y < pkt.first.y + pkt.second.b))
        {
            advanced_eps.push_back(potential_ep);
            hit_bottom = false;
            break;
        }
        // check if the pkt is being blocked by other packets
        if (start.x >= pkt.first.x)
        {
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_z)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.z > pkt.first.z + pkt.second.h)
                    continue;
                if (!((blocking_pkt.first.y + blocking_pkt.second.b > start.y) && (blocking_pkt.first.y <= pkt.first.y)))
                    continue;
                if (!((start.x >= blocking_pkt.first.x) && (start.x < blocking_pkt.first.x + blocking_pkt.second.l)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }
        else if (start.y >= pkt.first.y)
        {
            // give line by line
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_z)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.z > pkt.first.z + pkt.second.h)
                    continue;
                if (!((blocking_pkt.first.x + blocking_pkt.second.l > start.x) && (blocking_pkt.first.x <= pkt.first.x)))
                    continue;
                if (!((start.y >= blocking_pkt.first.y) && (start.y < blocking_pkt.first.y + blocking_pkt.second.b)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }
        else
        { // diagonal intersection
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_z)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.z > pkt.first.z + pkt.second.h)
                    continue;
                is_blocked = check_collision_2D(pkt.first.x, pkt.first.x + pkt.second.l,
                                                pkt.first.y, pkt.first.y + pkt.second.b,
                                                blocking_pkt.first.x, blocking_pkt.first.x + blocking_pkt.second.l,
                                                blocking_pkt.first.y, blocking_pkt.first.y + blocking_pkt.second.b);
                if (is_blocked)
                    break;
            }
            if (is_blocked)
                continue;
        }

        // add to the blocking pkts list
        ULD_blocking_boxes_z.insert(i);
        if (pkt.first.z + pkt.second.h < start.z)
        {
            advanced_eps.push_back(potential_ep);
        }
    }
    ULD_blocking_boxes_z.clear();
    if (hit_bottom)
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = start.y;
        potential_ep.z = 0;
        advanced_eps.push_back(potential_ep);
    }
    return;
}

void Solver::projection_neg_y_advanced(vector<coords> &advanced_eps, coords start, int self_pid)
{

    bool hit_bottom = true;
    for (auto i : ULD_sorted_y[start.box])
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = placement[i].first.y + placement[i].second.b;
        potential_ep.z = start.z;

        if (i == self_pid)
            continue;
        auto pkt = placement[i];
        if (pkt.first.x == -1)
            continue;
        if (pkt.first.x + pkt.second.l <= start.x)
            continue;
        if (pkt.first.z + pkt.second.h <= start.z)
            continue;
        if (pkt.first.y >= start.y)
            continue;

        // case 0: ray hits the object (no more projections)
        if ((start.x >= pkt.first.x) && (start.x < pkt.first.x + pkt.second.l) &&
            (start.z >= pkt.first.z) && (start.z < pkt.first.z + pkt.second.h))
        {
            advanced_eps.push_back(potential_ep);
            hit_bottom = false;
            break;
        }
        // check if the pkt is being blocked by other packets
        if (start.x >= pkt.first.x)
        {
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_y)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.y > pkt.first.y + pkt.second.b)
                    continue;
                if (!((blocking_pkt.first.z + blocking_pkt.second.h > start.z) && (blocking_pkt.first.z <= pkt.first.z)))
                    continue;
                if (!((start.x >= blocking_pkt.first.x) && (start.x < blocking_pkt.first.x + blocking_pkt.second.l)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }
        else if (start.z >= pkt.first.z)
        {
            // give line by line
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_y)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.y > pkt.first.y + pkt.second.b)
                    continue;
                if (!((blocking_pkt.first.x + blocking_pkt.second.l > start.x) && (blocking_pkt.first.x <= pkt.first.x)))
                    continue;
                if (!((start.z >= blocking_pkt.first.z) && (start.z < blocking_pkt.first.z + blocking_pkt.second.h)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }

        else
        { // diagonal intersection
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_y)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.x == -1)
                    continue; // although this should not happen
                is_blocked = check_collision_2D(pkt.first.x, pkt.first.x + pkt.second.l,
                                                pkt.first.z, pkt.first.z + pkt.second.h,
                                                blocking_pkt.first.x, blocking_pkt.first.x + blocking_pkt.second.l,
                                                blocking_pkt.first.z, blocking_pkt.first.z + blocking_pkt.second.h);
                if (is_blocked)
                    break;
            }
            if (is_blocked)
                continue;
        }

        // add to the blocking pkts list
        ULD_blocking_boxes_y.insert(i);
        if (pkt.first.y + pkt.second.b < start.y)
        {
            advanced_eps.push_back(potential_ep);
        }
    }
    ULD_blocking_boxes_y.clear();
    if (hit_bottom)
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = start.x;
        potential_ep.y = 0;
        potential_ep.z = start.z;
        advanced_eps.push_back(potential_ep);
    }
    return;
}

void Solver::projection_neg_x_advanced(vector<coords> &advanced_eps, coords start, int self_pid)
{

    bool hit_bottom = true;
    for (auto i : ULD_sorted_x[start.box])
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = placement[i].first.x + placement[i].second.l;
        potential_ep.y = start.y;
        potential_ep.z = start.z;

        if (i == self_pid)
            continue;
        auto pkt = placement[i];
        if (pkt.first.x == -1)
            continue;
        if (pkt.first.y + pkt.second.b <= start.y)
            continue;
        if (pkt.first.z + pkt.second.h <= start.z)
            continue;
        if (pkt.first.x >= start.x)
            continue;

        // case 0: ray hits the object (no more projections)
        if ((start.y >= pkt.first.y) && (start.y < pkt.first.y + pkt.second.b) &&
            (start.z >= pkt.first.z) && (start.z < pkt.first.z + pkt.second.h))
        {
            advanced_eps.push_back(potential_ep);
            hit_bottom = false;
            break;
        }
        // check if the pkt is being blocked by other packets
        if (start.y >= pkt.first.y)
        {
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_x)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.y == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.x > pkt.first.x + pkt.second.l)
                    continue;
                if (!((blocking_pkt.first.z + blocking_pkt.second.h > start.z) && (blocking_pkt.first.z <= pkt.first.z)))
                    continue;
                if (!((start.y >= blocking_pkt.first.y) && (start.y < pkt.first.y + pkt.second.b)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }
        else if (start.z >= pkt.first.z)
        {
            // give line by line
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_x)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.y == -1)
                    continue; // although this should not happen
                if (blocking_pkt.first.x > pkt.first.x + pkt.second.l)
                    continue;
                if (!((blocking_pkt.first.y + blocking_pkt.second.b > start.y) && (blocking_pkt.first.y <= pkt.first.y)))
                    continue;
                if (!((start.z >= blocking_pkt.first.z) && (start.z < blocking_pkt.first.z + blocking_pkt.second.h)))
                    continue;
                is_blocked = true;
                break;
            }
            if (is_blocked)
                continue;
        }
        else
        { // diagonal intersection
            bool is_blocked = false;
            for (auto j : ULD_blocking_boxes_x)
            {
                auto blocking_pkt = placement[j];
                if (blocking_pkt.first.y == -1)
                    continue; // although this should not happen
                is_blocked = check_collision_2D(pkt.first.y, pkt.first.y + pkt.second.b,
                                                pkt.first.z, pkt.first.z + pkt.second.h,
                                                blocking_pkt.first.y, blocking_pkt.first.y + blocking_pkt.second.b,
                                                blocking_pkt.first.z, blocking_pkt.first.z + blocking_pkt.second.h);
                if (is_blocked)
                    break;
            }
            if (is_blocked)
                continue;
        }

        // add to the blocking pkts list
        ULD_blocking_boxes_x.insert(i);
        if (pkt.first.x + pkt.second.l < start.x)
        {
            advanced_eps.push_back(potential_ep);
        }
    }
    ULD_blocking_boxes_x.clear();
    if (hit_bottom)
    {
        coords potential_ep;
        potential_ep.box = start.box;
        potential_ep.x = 0;
        potential_ep.y = start.y;
        potential_ep.z = start.z;
        advanced_eps.push_back(potential_ep);
    }
    return;
}
void Solver::addEP2(int i)
{
    coords ob1, ob2, ob3;
    auto p = placement[i];
    // p.first = def;
    int uid = p.first.box;
    ob1.box = uid;
    ob1.x = p.first.x;
    ob1.y = p.first.y + p.second.b;
    ob1.z = p.first.z + p.second.h;

    ob2.box = uid;
    ob2.x = p.first.x + p.second.l;
    ob2.y = p.first.y;
    ob2.z = p.first.z + p.second.h;

    ob3.box = uid;
    ob3.x = p.first.x + p.second.l;
    ob3.y = p.first.y + p.second.b;
    ob3.z = p.first.z;

    vector<coords> advanced_eps;
    projection_neg_y_advanced(advanced_eps, ob1, i);
    projection_neg_z_advanced(advanced_eps, ob1, i);

    projection_neg_x_advanced(advanced_eps, ob2, i);
    projection_neg_z_advanced(advanced_eps, ob2, i);

    projection_neg_x_advanced(advanced_eps, ob3, i);
    projection_neg_y_advanced(advanced_eps, ob3, i);

    for (auto x : advanced_eps)
    {
        auto r = getResidueSpace(x);
        if ((r.first > RESIDUE_THRESHOLD) and (r.second.first > RESIDUE_THRESHOLD) and (r.second.second) > RESIDUE_THRESHOLD)
            ep[convertCoords(x)] = r;
    }

    placement[i] = p;
}
void Solver::addEP(int i)
{
    coords ob1;
    auto p = placement[i];
    placement[i].first = def;
    ob1.box = p.first.box;
    ob1.x = p.first.x;
    ob1.y = p.first.y + placement[i].second.b;
    ob1.z = p.first.z + p.second.h;
    auto t = beamprojectYNeg(ob1);
    auto r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectZNeg(ob1);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    coords ob2;
    ob2.box = p.first.box;
    ob2.x = p.first.x + p.second.l;
    ob2.y = p.first.y;
    ob2.z = p.first.z + p.second.h;
    t = beamprojectXNeg(ob2);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectZNeg(ob2);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    coords ob3;
    ob3.box = p.first.box;
    ob3.x = p.first.x + p.second.l;
    ob3.y = p.first.y + p.second.b;
    ob3.z = p.first.z;
    t = beamprojectXNeg(ob3);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second >= RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    t = beamprojectYNeg(ob3);
    r = getResidueSpace(t);
    if (r.first > RESIDUE_THRESHOLD and r.second.first > RESIDUE_THRESHOLD and r.second.second > RESIDUE_THRESHOLD)
        ep[convertCoords(t)] = r;
    placement[i] = p;
}

void Solver::updateMaxBound(int i)
{
    int b = placement[i].first.box;
    ULDl[b].maxBound.x = max(ULDl[b].maxBound.x, placement[i].first.x + placement[i].second.l);
    ULDl[b].maxBound.y = max(ULDl[b].maxBound.y, placement[i].first.y + placement[i].second.b);
    ULDl[b].maxBound.z = max(ULDl[b].maxBound.z, placement[i].first.z + placement[i].second.h);
}

void Solver::updateResidue(int i)
{
    // update residual space also to be done
    auto it = ep.lower_bound(convertCoords(placement[i].first));
    int b = placement[i].first.box;
    while (it != ep.end() and it->first.first == b)
    {
        auto p = *it;
        coords e;
        e.box = p.first.first;
        e.x = p.first.second.first;
        e.y = p.first.second.second.first;
        e.z = p.first.second.second.second;
        int t = XBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.first = min(p.second.first, t);
        t = YBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.second.first = min(p.second.second.first, t);
        t = ZBeamIntersectionWithBox(e, i);
        if (t != -1)
            p.second.second.second = min(p.second.second.second, t);
        it++;
    }
}
int Solver::XBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
        {
            auto p = start;
            p.y = min(p.y + i, ULDl[b].dim.b - 1);
            p.z = min(p.z + j, ULDl[b].dim.h - 1);
            int t = XRayIntersectionWithBox(p, ind);
            if (t != -1)
            {
                r = min(r, t);
            }
        }
    return r == INF ? -1 : r;
}
int Solver::YBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
        {
            auto p = start;
            p.x = min(p.x + i, ULDl[b].dim.l - 1);
            p.z = min(p.z + j, ULDl[b].dim.h - 1);
            int t = YRayIntersectionWithBox(p, ind);
            if (t != -1)
            {
                r = min(r, t);
            }
        }
    return r == INF ? -1 : r;
}
int Solver::ZBeamIntersectionWithBox(coords start, int ind)
{
    int r = INF;
    int b = start.box;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
        {
            auto p = start;
            p.x = min(p.x + i, ULDl[b].dim.l - 1);
            p.y = min(p.y + j, ULDl[b].dim.b - 1);
            int t = ZRayIntersectionWithBox(p, ind);
            if (t != -1)
            {
                r = min(r, t);
            }
        }
    return r == INF ? -1 : r;
}
int Solver::XRayIntersectionWithBox(coords start, int ind)
{
    if (start.y < placement[ind].first.y + placement[ind].second.b and start.y > placement[ind].first.y and start.z < placement[ind].first.z + placement[ind].second.h and start.z > placement[ind].first.z and start.x < placement[ind].first.x + placement[ind].second.l and start.box == placement[ind].first.box)
        return placement[ind].first.x;
    return -1;
}
int Solver::YRayIntersectionWithBox(coords start, int ind)
{
    if (start.x < placement[ind].first.x + placement[ind].second.l and start.x > placement[ind].first.x and start.z < placement[ind].first.z + placement[ind].second.h and start.z > placement[ind].first.z and start.y < placement[ind].first.y + placement[ind].second.b and start.box == placement[ind].first.box)
        return placement[ind].first.y;
    return -1;
}
int Solver::ZRayIntersectionWithBox(coords start, int ind)
{
    if (start.x < placement[ind].first.x + placement[ind].second.l and start.x > placement[ind].first.x and start.y < placement[ind].first.y + placement[ind].second.b and start.y > placement[ind].first.y and start.z < placement[ind].first.z + placement[ind].second.h and start.box == placement[ind].first.box)
        return placement[ind].first.z;
    return -1;
}
coords Solver::rayProjectXNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.y <= placement[i].first.y + placement[i].second.b and start.y >= placement[i].first.y and start.z <= placement[i].first.z + placement[i].second.h and start.z >= placement[i].first.z and start.x >= placement[i].first.x)
            x = max(x, placement[i].first.x + placement[i].second.l);
    }
    start.x = min(start.x, x);
    return start;
}
coords Solver::rayProjectYNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x <= placement[i].first.x + placement[i].second.l and start.x >= placement[i].first.x and start.z <= placement[i].first.z + placement[i].second.h and start.z >= placement[i].first.z and start.y >= placement[i].first.y)
            x = max(x, placement[i].first.y + placement[i].second.b);
    }
    start.y = min(start.y, x);
    return start;
}
coords Solver::rayProjectZNeg(coords start)
{
    int x = 0;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x <= placement[i].first.x + placement[i].second.l and start.x >= placement[i].first.x and start.y <= placement[i].first.y + placement[i].second.b and start.y >= placement[i].first.y and start.z >= placement[i].first.z)
            x = max(x, placement[i].first.z + placement[i].second.h);
    }
    start.z = min(start.z, x);
    return start;
}
coords Solver::rayProjectXPos(coords start)
{
    int x = ULDl[start.box].dim.l;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.y < placement[i].first.y + placement[i].second.b and start.y > placement[i].first.y and start.z < placement[i].first.z + placement[i].second.h and start.z > placement[i].first.z and start.x < placement[i].first.x + placement[i].second.l)
            x = min(x, placement[i].first.x);
    }
    start.x = max(start.x, x);
    return start;
}
coords Solver::rayProjectYPos(coords start)
{
    int x = ULDl[start.box].dim.b;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x < placement[i].first.x + placement[i].second.l and start.x > placement[i].first.x and start.z < placement[i].first.z + placement[i].second.h and start.z > placement[i].first.z and start.y < placement[i].first.y + placement[i].second.b)
            x = min(x, placement[i].first.y);
    }
    start.y = max(start.y, x);
    return start;
}
coords Solver::rayProjectZPos(coords start)
{
    int x = ULDl[start.box].dim.h;
    for (int i : ULDPackages[start.box])
    {
        if (placement[i].first.x != -1 and start.x < placement[i].first.x + placement[i].second.l and start.x > placement[i].first.x and start.y < placement[i].first.y + placement[i].second.b and start.y > placement[i].first.y and start.z < placement[i].first.z + placement[i].second.h)
            x = min(x, placement[i].first.z);
    }
    start.z = max(start.z, x);
    return start;
}
bool Solver::checkGravity(coords e, Box b)
{
    // If the z-coordinate is at the ground level, gravity does not apply
    if (e.z == 0)
        return false;

    // Define the bounding box of the current element
    std::pair<int, int> x1_min = {e.x, e.y};
    std::pair<int, int> x1_max = {e.x + b.l, e.y + b.b};

    int overlappingSurfacesCount = 0;

    // Count the number of surfaces at the same box and z-coordinate
    for (const auto &surface : surfaces[e.box])
    {
        if (surface.first == e.z)
            overlappingSurfacesCount++;
    }

    // If no overlapping surfaces are found at the z-coordinate, gravity applies
    if (overlappingSurfacesCount == 0)
        return true;

    // Check for overlapping surfaces with the current box
    for (const auto &surface : surfaces[e.box])
    {
        if (surface.first != e.z)
            continue;

        auto [z, coords] = surface;
        auto [minCoords, maxCoords] = coords;

        std::pair<int, int> x2_min = minCoords;
        std::pair<int, int> x2_max = maxCoords;

        // Check for overlap between bounding boxes
        if (x1_max.first > x2_min.first && x1_min.first < x2_max.first &&
            x1_max.second > x2_min.second && x1_min.second < x2_max.second)
        {
            return false; // Overlap found, gravity does not apply
        }
    }

    return true; // No overlap, gravity applies
}
void Solver::gravity_pull(int i)
{
    pair<int, int> x1_min = make_pair(placement[i].first.x, placement[i].first.y), x1_max = {placement[i].first.x + placement[i].second.l, placement[i].first.y + placement[i].second.b};
    int m = 0;
    for (auto x : surfaces[placement[i].first.box])
    {
        pair<int, pair<pair<int, int>, pair<int, int>>> p = x;
        pair<int, int> x2_min = make_pair(p.second.first.first, p.second.first.second), x2_max = {p.second.second.first, p.second.second.second};
        if (x1_max.first > x2_min.first && x1_min.first < x2_max.first && x1_max.second > x2_min.second && x1_min.second < x2_max.second && x.first <= placement[i].first.z)
            m = max(m, x.first);
    }
    placement[i].first.z = m;
}

int residueFunc(coords c, Box b, Solver *s)
{
    int r = 0;
    r += (1LL * (s->ULDHasPriority[c.box]) * 100000000000LL) * b.isPriority;
    float relativeDifference = (s->ep[convertCoords(c)].first - b.l) / 1.0 / s->ep[convertCoords(c)].first + (s->ep[convertCoords(c)].second.first - b.b) / 1.0 / s->ep[convertCoords(c)].second.first + weightz * (s->ep[convertCoords(c)].second.second - b.h) / 1.0 / s->ep[convertCoords(c)].second.second;
    relativeDifference *= 1000000;
    //    float relativeDifference =(s->ep[convertCoords(c)].first - b.l)+(s->ep[convertCoords(c)].second.first - b.b)+0.1*(s->ep[convertCoords(c)].second.second - b.h);
    r += relativeDifference;
    return r;
}
void ScoredSolver::solve()
{
    // Initial Solve
    insertionCounter.assign(data.size() + 5, 0);
#ifndef GENETIC
#ifdef OLD_SORT
    sort(data.begin(), data.end(), this->sorter.val);
#endif
#ifndef OLD_SORT
    this->sorter.val(data);
#endif
#endif
#ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for (auto it : data)
    {
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
#endif
    for(int i=0;i<ULDl.size();i++) ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
    for(int i=0;i<data.size();i++)
    {
        // Construct Economy Package and Box Map
        if (!data[i].isPriority)
        {
            boxMap[data[i].ID] = &data[i];
            // cout << "Box ID: " << data[i].ID << endl;
            economyPackages.insert(data[i].ID);
        }

        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h;
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if (checkCollision(e, p))
                    continue;
                p.isPriority = b.isPriority;
                int score = this->merit.val(e, p, this);
                if (best.first < score)
                    best = pair<int, pair<coords, Box>>(score, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        insertionCounter[data[i].ID] += 1;
        if (data[i].isPriority)
        {
            ULDHasPriority[best.second.first.box] = true;
        }
        else
        {
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        // ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        addEP2(i);
        // addEP(i);
        update(i);
    }

    // Initialising the scores
    // double k = 2.0;
    // for(auto i: economyPackages){
    //     // if(lastInsertionSet.find(i) == lastInsertionSet.end()){
    //     if(0){
    //         score[i] = k*boxMap[i]->cost;
    //     }
    //     else{
    //         score[i] = boxMap[i]->cost;
    //     }
    // }
    cout << "Base Solution Cost: " << this->cost() << endl;
#ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for (auto it : data)
    {
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
#endif
    bestCost = this->cost();
    // cout << bestCost;
    // return;
    bestSolution = lastInsertion;
    // optimize(0);
    // writeToFile("other_result.csv");
    // cout << "New Cost: " << this->cost() << endl;
    // assert(this->cost() == bestCost);
    int lastChangeIter = -1;
    int reinitializeIter = 100, noChangeThreshold = 10;
    int solverTime = static_cast<int>(time(nullptr));
    for (int i = 1; i <= iterations; i++)
    {
        cout << "Iteration " << i << " started" << endl;
        if (i % reinitializeIter == 1 || i - lastChangeIter >= noChangeThreshold)
        {
            cout << "Reinitializing at iteration " << i << endl;
            reinitialize(true, 2.0, 25);
            lastChangeIter = i;
        }
        else
        {
            update_scores(i);
        }
        // Solver new_solver(this->sorter, this->merit, this->data, this->originalUldList);
        // new_solver.solve();
        // cout << "New solver costs me " << new_solver.cost() << endl;
        optimize(i);
        cout << "Iteration " << i << " had a cost " << this->cost() << endl;
        if (this->cost() > bestCost)
        {
            cout << "Found new solution at iteration " << i << " with cost " << this->cost() << endl;
            stringstream ss;
            ss << "result_" << i << "_" << solverTime << ".txt";
            this->writeToFile(ss.str());
            bestCost = this->cost();
            lastChangeIter = i;
            bestSolution = lastInsertion;
            cout << "Updated base solution to " << endl;
            for (auto it : bestSolution)
            {
                cout << it << " ";
            }
        }
    }
}

void ScoredSolver::reinitialize(bool _swap, double k, int num_swap)
{
    set<int> bestSolutionSet(bestSolution.begin(), bestSolution.end());
    vector<int> unselectedPackages;
    for (auto it : economyPackages)
    {
        if (bestSolutionSet.find(it) != bestSolutionSet.end())
        {
            score[it] = k * boxMap[it]->cost;
        }
        else
        {
            unselectedPackages.push_back(it);
            score[it] = boxMap[it]->cost;
        }
    }

    vector<int> economyPackagesVec(economyPackages.begin(), economyPackages.end());
    if (_swap)
    {
        for (int i = 0; i < num_swap; i++)
        {
            mt19937 mt(time(nullptr));
#define SWAP_DIFFERENT
#ifndef SWAP_DIFFERENT
            int idx, idy;
            do
            {
                idx = mt() % economyPackagesVec.size();
                idy = mt() % economyPackagesVec.size();
            } while (idx == idy);
            swap(score[economyPackagesVec[idx]], score[economyPackagesVec[idy]]);
#endif
#ifdef SWAP_DIFFERENT
            int idx, idy;
            do
            {
                idx = mt() % bestSolution.size();
                idy = mt() % unselectedPackages.size();
            } while (idx == idy);
            swap(score[bestSolution[idx]], score[unselectedPackages[idy]]);
#endif
        }
    }
}

/*
 * i = number of iterations completed
 */
void ScoredSolver::update_scores(int i)
{

    pair<double, int> worst_loaded = {100000000.0, -1}, best_unloaded = {0.0, -1};
    for (auto id : economyPackages)
    {
        if (lastInsertionSet.find(id) == lastInsertionSet.end())
        {
            double theta = (1.0 * boxMap[id]->cost) / (boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1 + i - insertionCounter[id]));
            // cout << "Theta: "<< theta << " " << id << endl;
            if (theta > best_unloaded.first)
            {
                best_unloaded = {theta, id};
            }
        }
        else
        {
            double myu = (1.0 * boxMap[id]->cost) / (boxMap[id]->l * boxMap[id]->b * boxMap[id]->h * (1 + insertionCounter[id]));
            if (myu < worst_loaded.first)
            {
                worst_loaded = {myu, id};
            }
        }
    }
    // Found highest myu and theta
    // cout << "Reached here" << endl;
    // cout.flush();
    if (worst_loaded.second != -1 && best_unloaded.second != -1)
    {
        score[worst_loaded.second] *= (0.5);
        score[best_unloaded.second] *= (1.5);

        cout << "Score of " << best_unloaded.second << ":" << score[best_unloaded.second] << endl;
        swap(score[worst_loaded.second], score[best_unloaded.second]);
        cout << "Swapped " << worst_loaded.second << " and " << best_unloaded.second << "\n";
        cout << "Score of " << best_unloaded.second << ":" << score[best_unloaded.second] << endl;
    }
    else
    {
        cout << "No possible swaps " << worst_loaded.second << " " << best_unloaded.second << endl;
        return;
    }
}

void ScoredSolver::optimize(int _iter)
{
    // Re-Setup the solver
    placement.clear();
    ULDPackages.clear();
    surfaces.clear();
    ULDHasPriority.clear();
    lastInsertion.clear();
    lastInsertionSet.clear();
    def.x = def.y = def.z = def.box = -1;
    Box def_;
    def_.l = def_.b = def_.h = -1;
    ULDl = this->originalUldList;
    placement.assign(data.size(), pair<coords, Box>(def, def_));
    ULDPackages.assign(ULDl.size(), set<int>());
    ULD_sorted_x.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_x));
    ULD_sorted_y.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_y));
    ULD_sorted_z.assign(ULDl.size(), set<int, function<bool(const int &, const int &)>>(compare_z));
    ULD_blocking_boxes_x = set<int, function<bool(const int &, const int &)>>(compare_x_base);
    ULD_blocking_boxes_y = set<int, function<bool(const int &, const int &)>>(compare_y_base);
    ULD_blocking_boxes_z = set<int, function<bool(const int &, const int &)>>(compare_z_base);
    surfaces.assign(ULDl.size(), set<pair<int, pair<pair<int, int>, pair<int, int>>>>());
    ULDHasPriority.assign(ULDl.size(), false);
    ep.clear();

// sort(data.begin(), data.end(), this->sorter.val);
#define SORT_WITH_MARKS
#ifdef SORT_WITH_MARKS
    vector<int> mark(data.size() + 5);
    for (int i = 0; i != data.size(); i++)
    {
        for (int j = 0; j != data.size(); j++)
        {
            if (data[i].isPriority || data[j].isPriority)
                continue;
            if (data[i].cost < data[j].cost)
                continue;
            vector<int> perm_i = {data[i].l, data[i].b, data[i].h};
            sort(perm_i.begin(), perm_i.end());
            vector<int> perm_j = {data[j].l, data[j].b, data[j].h};
            sort(perm_j.begin(), perm_j.end());
            if (perm_i[0] <= perm_j[0] && perm_i[1] <= perm_j[1] && perm_i[2] <= perm_j[2])
            {
                if (!(perm_i[0] == perm_j[0] && perm_i[1] == perm_j[1] && perm_i[2] == perm_j[2]) && data[i].cost == data[j].cost)
                { // Ensure they don't correspond to the samme box either
                    mark[data[j].ID]++;
                    break;
                }
            }
        }
    }

    sort(data.begin(), data.end(), [&](Box a, Box b)
         {
        if(b.isPriority and (not a.isPriority))return false;
        if(a.isPriority and (not b.isPriority))return true;
        // if(mark[a.ID] != mark[b.ID]) return mark[a.ID] < mark[b.ID];
        // if(a.isPriority and b.isPriority)return false;
        // if(a.cost!=b.cost)return a.cost>b.cost;
        int vol1 = a.l*a.b*a.h, vol2 = b.l*b.b*b.h;
        
            // double power_fac = 1.0;
        double fac1 = pow(score[a.ID], power_fac)/(vol1*1.0), fac2 = pow(score[b.ID], power_fac)/(vol2*1.0);
        if(score[a.ID]!=score[b.ID])return fac1 > fac2;

        // if(score[a.ID]!=score[b.ID])return score[a.ID]>score[b.ID];
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h; });
#endif
#ifndef SORT_WITH_MARKS
    sort(data.begin(), data.end(), [&](Box a, Box b)
         {
        if (a.isPriority && !b.isPriority)
            return true;
        if (!a.isPriority && b.isPriority)
            return false;
        // if(a.cost != b.cost) return a.cost > b.cost;

        if(score[a.ID]!=score[b.ID])return score[a.ID]>score[b.ID];
        if(a.l*a.b*a.h==b.l*b.b*b.h)return min(a.h,min(a.b,a.l))<min(b.h,min(b.b,b.l));
        return a.l*a.b*a.h > b.l*b.b*b.h; });
#endif

// this->sorter.val(data);
#ifdef DATA_ORDERING
    cout << "Data ordering:" << endl;
    for (auto it : data)
    {
        // cout << it.ID << "," << score[it.ID] << " ";
        cout << it.ID << " ";
    }
    cout << endl;
#endif
    For(i, ULDl.size()) ep[pair<int, pair<int, pii>>(i, pair<int, pii>(0, pii(0, 0)))] = pair<int, pii>(ULDl[i].dim.l, pii(ULDl[i].dim.b, ULDl[i].dim.h));
    For(i, data.size())
    {
        // Construct Economy Package and Box Map
        Box b = data[i];
        pair<int, pair<coords, Box>> best;
        best.first = -INF;
        vector<Box> perms(6);
        perms[0].l = b.l;
        perms[0].b = b.b;
        perms[0].h = b.h;
        perms[1].l = b.l;
        perms[1].h = b.b;
        perms[1].b = b.h;
        perms[2].l = b.b;
        perms[2].b = b.l;
        perms[2].h = b.h;
        perms[3].l = b.b;
        perms[3].b = b.h;
        perms[3].h = b.l;
        perms[4].l = b.h;
        perms[4].b = b.b;
        perms[4].h = b.l;
        perms[5].l = b.h;
        perms[5].b = b.l;
        perms[5].h = b.b;
        for (Box p : perms)
            for (auto x : ep)
            {
                // check if it can fit
                coords e;
                e.box = x.first.first;
                e.x = x.first.second.first;
                e.y = x.first.second.second.first;
                e.z = x.first.second.second.second;
                if (checkCollision(e, p))
                    continue;
                p.isPriority = b.isPriority;
                int scores = this->merit.val(e, p, this);
                if (best.first < scores)
                    best = pair<int, pair<coords, Box>>(scores, pair<coords, Box>(e, p));
            }
        // update vals
        if (best.first == -INF)
            continue;
        insertionCounter[data[i].ID] += 1;
        if (data[i].isPriority)
        {
            ULDHasPriority[best.second.first.box] = true;
        }
        else
        {
            lastInsertionSet.insert(data[i].ID);
            lastInsertion.push_back(data[i].ID);
        }
        placement[i] = best.second;
        ULDPackages[best.second.first.box].insert(i);
        ULD_sorted_x[best.second.first.box].insert(i);
        ULD_sorted_y[best.second.first.box].insert(i);
        ULD_sorted_z[best.second.first.box].insert(i);
        // int pp=placement[i].first.z;
        gravity_pull(i);
        // if(pp!=placement[i].first.z)
        // c++;
        addEP2(i);
        // addEP(i);
        update(i);
        // if(placement[i].second==data[i])
        // if(checkGravity(placement[i].first, placement[i].second))
        // printf("error\n");
    }
}