#include<iostream>
#include<map>
#include<utility>
using namespace std;

int main(){
    map<int ,pair<char,int>> mp;
    mp[0] = pair<char,int>('a',1);
    mp[1] = pair<char,int>('b',2);
    mp[2] = pair<char,int>('c',3);
    mp[3] = pair<char,int>('d',4);
    map<int,pair<char,int>>mp2;
    mp2 = map<int,pair<char,int>>(mp);
    for(map<int ,pair<char,int>>::iterator it = mp2.begin();it!=mp2.end();it++ ){
        cout<<it->first<<' '<<it->second.first<<' '<<it->second.second<<endl;
    }
    return 0;

}