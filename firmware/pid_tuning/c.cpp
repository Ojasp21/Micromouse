#include<iostream>
#include<string>
#include<vector>
using namespace std;
void printsubarray(int a[],int n){
    for(int i=0;i<n;i++){
        for(int j=i;j<n;j++){
            cout<<"("<<a[i]<<","<<a[j]<<")";
        }
        cout<<endl;
    }
}
int main(){
     int maze[16][16]={
        {31,30,29,28,27,26,25,24,25,24,23,22,21,20,21,22},
        {32,33,48,49,50,51,26,23,22,19,18,23,22,19,20,23},
        {33,34,47,50,51,52,25,24,21,20,17,16,17,18,19,24},
        {34,35,46,45,46,53,24,23,22,19,18,15,16,17,18,25},
        {35,36,37,44,55,54,23,22,21,20,13,14,15,16,17,26},
        {36,37,38,43,42,55, 8, 9,10,11,12,13,14,15,28,27},
        {39,38,39,40,41, 6, 7, 8, 9,10,13,12,13,30,29,28},
        {40,39,40,47,50, 5, 4, 0, 0,11,10,11,14,31,30,31},
        {41,40,41,46,49, 4, 3, 0, 0, 8, 9,12,37,36,33,32},
        {58,41,42,45,48, 3, 2, 1, 6, 7,10,39,38,35,34,33},
        {57,44,43,44,47,48, 3, 4, 5, 8,41,40,39,36,35,34},
        {56,57,58,45,46,49,52,53,46,45,42,41,38,37,36,35},
        {55,56,57,48,47,50,51,52,47,44,43,40,39,38,37,36},
        {54,55,50,49,48,51,50,51,48,47,42,41,40,41,38,37},
        {53,52,51,52,49,50,49,48,49,46,43,44,41,40,39,38},
        {54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,41}
    };
    int x=0;
    int y=15;
    cout<<maze[y][x];
int a[3][2]={{1,2},
             {3,4},
             {5,6}};
//cout<<a[2][0];

}