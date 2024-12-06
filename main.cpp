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
#define PRIORITY_MISS_COST 1000000
#define NEW_ULD_PRIORITY_COST 5000
#define RESIDUE_THRESHOLD 0
#define convertCoords(pt) pair<int,pair<int,pii>>(pt.box,pair<int,pii>(pt.x,pii(pt.y,pt.z)))
using namespace std;
vector<Uld> ULDList(6);
vector<Box>dat(400);

double weightz = 0.2;
double power_fac = 1.0;

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
    double weightz_min = 0.1, power_fac_min = 3.7;
    // vector<Box> original_dat = dat; 
    // Output results
    // freopen("result.csv", "w", stdout);

    Sorter Final_Ht = Ashish_Ht;
    
    // for(weightz = 0.1; weightz <= 0.1; weightz += 0.02){
    //     for(power_fac= 3; power_fac <= 4; power_fac += 0.02){
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
    Sorter emptySorter;
    emptySorter.val = [](vector<Box> &data){
        return;
    };
    // vector<int> idOrdering = {365, 165, 133, 394, 105, 356, 274, 211, 297, 134, 300, 283, 41, 145, 346, 139, 277, 333, 215, 255, 299, 142, 73, 129, 36, 282, 45, 125, 10, 20, 168, 252, 319, 50, 273, 183, 9, 82, 2, 106, 264, 46, 94, 285, 6, 157, 366, 217, 219, 295, 284, 224, 212, 124, 329, 187, 238, 350, 103, 68, 109, 163, 180, 112, 148, 126, 181, 360, 226, 136, 162, 304, 207, 17, 280, 214, 236, 266, 199, 353, 16, 23, 42, 49, 232, 222, 275, 71, 310, 15, 216, 188, 3, 69, 320, 352, 86, 117, 270, 147, 265, 345, 150, 1, 272, 393, 324, 387, 92, 80, 385, 37, 357, 258, 185, 220, 57, 378, 26, 156, 164, 32, 291, 167, 75, 135, 327, 97, 380, 313, 29, 27, 25, 4, 218, 335, 336, 189, 102, 307, 205, 76, 202, 257, 305, 190, 398, 315, 371, 87, 171, 166, 58, 170, 172, 246, 208, 78, 61, 33, 247, 169, 363, 326, 201, 369, 5, 248, 12, 93, 391, 186, 261, 59, 174, 28, 192, 206, 227, 269, 65, 40, 198, 396, 386, 122, 130, 382, 312, 99, 39, 175, 400, 322, 351, 152, 44, 234, 337, 194, 278, 367, 242, 19, 120, 343, 244, 54, 146, 317, 177, 381, 230, 355, 331, 85, 384, 375, 289, 256, 397, 179, 149, 151, 221, 123, 392, 14, 48, 53, 141, 399, 7, 8, 11, 13, 18, 21, 22, 24, 30, 31, 34, 35, 38, 43, 47, 51, 52, 55, 56, 60, 62, 63, 64, 66, 67, 70, 72, 74, 77, 79, 81, 83, 84, 88, 89, 90, 91, 95, 96, 98, 100, 101, 104, 107, 108, 110, 111, 113, 114, 115, 116, 118, 119, 121, 127, 128, 131, 132, 137, 138, 140, 143, 144, 153, 154, 155, 158, 159, 160, 161, 173, 176, 178, 182, 184, 191, 193, 195, 196, 197, 200, 203, 204, 209, 210, 213, 223, 225, 228, 229, 231, 233, 235, 237, 239, 240, 241, 243, 245, 249, 250, 251, 253, 254, 259, 260, 262, 263, 267, 268, 271, 276, 279, 281, 286, 287, 288, 290, 292, 293, 294, 296, 298, 301, 302, 303, 306, 308, 309, 311, 314, 316, 318, 321, 323, 325, 328, 330, 332, 334, 338, 339, 340, 341, 342, 344, 347, 348, 349, 354, 358, 359, 361, 362, 364, 368, 370, 372, 373, 374, 376, 377, 379, 383, 388, 389, 390, 395};
    // vector<int> idOrdering = {365, 165, 346, 394, 300, 105, 274, 356, 297, 134, 211, 145, 41, 283, 255, 215, 277, 333, 139, 133, 299, 142, 73, 129, 36, 282, 68, 125, 10, 20, 168, 252, 319, 50, 366, 285, 9, 82, 124, 106, 264, 219, 94, 183, 6, 2, 273, 217, 46, 157, 284, 224, 212, 295, 329, 187, 163, 350, 117, 45, 226, 238, 180, 236, 148, 126, 181, 360, 109, 136, 162, 304, 207, 15, 352, 214, 112, 266, 199, 353, 16, 23, 42, 49, 232, 222, 275, 71, 280, 17, 216, 188, 3, 69, 86, 265, 320, 103, 310, 147, 270, 345, 150, 1, 272, 393, 324, 387, 326, 92, 80, 385, 378, 37, 357, 258, 220, 57, 185, 26, 291, 167, 75, 135, 169, 363, 27, 380, 25, 313, 97, 29, 327, 164, 336, 335, 32, 4, 189, 247, 102, 205, 202, 76, 307, 166, 315, 305, 156, 382, 398, 242, 87, 171, 172, 170, 257, 58, 5, 371, 78, 208, 12, 61, 33, 343, 28, 218, 248, 174, 391, 261, 201, 93, 206, 65, 194, 59, 154, 278, 186, 190, 386, 198, 19, 240, 244, 312, 396, 120, 39, 221, 369, 341, 40, 351, 99, 44, 231, 337, 246, 192, 322, 123, 122, 375, 130, 384, 146, 54, 43, 317, 381, 331, 339, 230, 227, 271, 269, 234, 177, 355, 289, 397, 77, 108, 149, 400, 392, 66, 14, 276, 53, 85, 243, 249, 309, 321, 63, 140, 144, 121, 137, 334, 107, 35, 118, 114, 131, 116, 155, 115, 11, 89, 21, 52, 101, 253, 364, 79, 348, 151, 281, 7, 113, 81, 152, 195, 268, 8, 104, 62, 303, 293, 209, 128, 48, 31, 70, 34, 361, 60, 56, 141, 84, 138, 132, 376, 372, 96, 91, 110, 153, 83, 100, 88, 13, 72, 159, 22, 349, 160, 55, 127, 98, 95, 24, 51, 161, 158, 74, 119, 67, 47, 143, 30, 90, 111, 38, 18, 64, 223, 358, 286, 250, 298, 370, 362, 314, 374, 302, 338, 290, 254, 213, 179, 184, 182, 191, 178, 388, 176, 399, 175, 260, 316, 193, 379, 256, 340, 318, 210, 237, 390, 197, 389, 204, 203, 342, 196, 200, 267, 233, 294, 354, 332, 368, 292, 279, 377, 259, 328, 306, 330, 344, 308, 296, 367, 262, 173, 325, 235, 245, 383, 373, 359, 263, 347, 323, 228, 229, 395, 251, 241, 301, 288, 311, 287, 225, 239};
    // vector<int> idOrdering = {365, 165, 346, 394, 300, 105, 274, 356, 297, 134, 211, 145, 41, 283, 255, 215, 277, 333, 139, 133, 299, 142, 73, 129, 36, 282, 68, 125, 10, 20, 168, 252, 319, 50, 366, 285, 9, 82, 124, 106, 264, 219, 94, 183, 6, 2, 273, 217, 46, 157, 284, 224, 212, 295, 329, 187, 163, 350, 117, 45, 226, 238, 180, 236, 148, 126, 181, 360, 109, 136, 162, 304, 207, 15, 352, 214, 112, 266, 199, 353, 16, 23, 42, 49, 232, 222, 275, 71, 280, 17, 216, 188, 3, 69, 86, 265, 320, 103, 310, 147, 270, 345, 150, 1, 272, 393, 324, 387, 326, 92, 80, 385, 378, 37, 357, 258, 220, 57, 185, 26, 291, 167, 75, 135, 169, 363, 27, 380, 25, 313, 97, 29, 327, 164, 336, 335, 32, 4, 189, 247, 102, 205, 202, 76, 307, 166, 315, 305, 156, 382, 398, 242, 87, 171, 172, 170, 257, 58, 5, 371, 78, 208, 12, 61, 33, 343, 28, 218, 248, 174, 391, 261, 201, 93, 206, 65, 194, 59, 154, 278, 186, 190, 386, 198, 19, 240, 244, 312, 396, 120, 39, 221, 369, 341, 40, 62, 99, 44, 231, 337, 246, 192, 322, 123, 122, 375, 130, 384, 146, 54, 11, 381, 331, 339, 230, 227, 271, 269, 234, 177, 355, 289, 397, 77, 108, 149, 400, 392, 66, 14, 276, 53, 85, 249, 317, 152, 351, 209, 48, 361, 141, 132, 159, 119, 399, 7, 8, 13, 18, 21, 22, 24, 30, 31, 34, 35, 38, 43, 47, 51, 52, 55, 56, 60, 63, 64, 67, 70, 72, 74, 79, 81, 83, 84, 88, 89, 90, 91, 95, 96, 98, 100, 101, 104, 107, 110, 111, 113, 114, 115, 116, 118, 121, 127, 128, 131, 137, 138, 140, 143, 144, 151, 153, 155, 158, 160, 161, 173, 175, 176, 178, 179, 182, 184, 191, 193, 195, 196, 197, 200, 203, 204, 210, 213, 223, 225, 228, 229, 233, 235, 237, 239, 241, 243, 245, 250, 251, 253, 254, 256, 259, 260, 262, 263, 267, 268, 279, 281, 286, 287, 288, 290, 292, 293, 294, 296, 298, 301, 302, 303, 306, 308, 309, 311, 314, 316, 318, 321, 323, 325, 328, 330, 332, 334, 338, 340, 342, 344, 347, 348, 349, 354, 358, 359, 362, 364, 367, 368, 370, 372, 373, 374, 376, 377, 379, 383, 388, 389, 390, 395};
    // vector<int> idOrdering = {365, 165, 215, 46, 300, 211, 274, 356, 346, 134, 105, 283, 41, 145, 133, 297, 299, 124, 212, 277, 255, 142, 73, 129, 36, 282, 45, 125, 10, 20, 168, 252, 319, 50, 273, 295, 9, 82, 2, 106, 264, 224, 394, 183, 6, 157, 366, 217, 94, 333, 284, 219, 139, 285, 265, 187, 238, 103, 304, 68, 109, 329, 180, 112, 148, 126, 181, 360, 226, 136, 162, 350, 207, 17, 280, 214, 236, 266, 199, 353, 16, 188, 42, 49, 232, 222, 163, 71, 352, 15, 216, 23, 3, 69, 320, 270, 86, 117, 310, 147, 275, 345, 150, 1, 272, 393, 324, 387, 326, 92, 80, 385, 378, 37, 357, 258, 220, 57, 185, 26, 291, 167, 75, 135, 169, 363, 327, 380, 25, 313, 97, 29, 27, 335, 336, 164, 32, 4, 189, 247, 102, 205, 202, 76, 307, 257, 315, 305, 156, 382, 398, 242, 87, 171, 172, 170, 166, 58, 218, 371, 78, 208, 12, 61, 33, 201, 227, 40, 248, 93, 391, 261, 343, 174, 192, 65, 194, 59, 154, 28, 186, 190, 386, 198, 19, 240, 244, 312, 396, 99, 39, 269, 369, 341, 322, 351, 120, 44, 234, 337, 246, 206, 221, 123, 122, 375, 130, 384, 146, 54, 43, 317, 381, 331, 339, 230, 278, 77, 5, 231, 177, 355, 289, 397, 14, 108, 149, 400, 392, 66, 85, 276, 53, 271, 121, 131, 11, 21, 63, 140, 144, 137, 107, 35, 118, 114, 116, 155, 115, 89, 52, 101, 178, 79, 151, 7, 113, 81, 152, 8, 104, 62, 128, 48, 91, 31, 70, 34, 60, 56, 141, 84, 138, 132, 96, 110, 153, 83, 100, 228, 88, 13, 213, 127, 72, 159, 22, 160, 55, 98, 95, 24, 47, 51, 161, 143, 74, 111, 38, 30, 64, 158, 119, 67, 90, 18, 175, 195, 203, 223, 182, 210, 225, 200, 204, 209, 193, 179, 196, 197, 176, 184, 191, 173, 395, 364, 372, 303, 359, 250, 239, 354, 298, 342, 253, 279, 301, 245, 233, 316, 347, 377, 267, 287, 256, 399, 259, 262, 332, 306, 288, 308, 263, 235, 323, 249, 293, 370, 374, 367, 348, 376, 344, 361, 241, 251, 338, 379, 309, 237, 311, 334, 292, 296, 358, 290, 286, 314, 368, 362, 389, 321, 340, 383, 330, 318, 254, 243, 390, 373, 328, 325, 349, 281, 268, 260, 294, 302, 388, 229};
    // cout << idOrdering.size();
    // return;
    ScoredSolver s(Final_Ht, Residue, dat, ULDList, 0);
    s.solve();
    // s.createCachedSolver(dat.size());
    cout << s.cost() << endl;
    return;






    // ScoredSolver s_first(Final_Ht, Residue, dat, ULDList, 0);
    
    // s_first.solve();
    // auto ordering = s_first.data;
    // vector<int> final_ordering;
    // for(auto it: ordering){
    //     final_ordering.push_back(it.ID);
    // }

    // ScoredSolver s(emptySorter, Residue, ordering, ULDList, 0);
    // // s.arrangeDataFromIDVector(final_ordering);
    // // s.arrangeDataFromIDVector(idOrdering);
    // s.solve();
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
}

signed main(){
    ios_base::sync_with_stdio(0); cin.tie(0);cout.tie();
    final_execution();
}
