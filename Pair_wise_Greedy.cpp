#include<bits/stdc++.h>
using namespace std;
//#define d2r (M_PI / 180.0)
const double d2r=0.0174532925199433;
    // Declare ifstream objects
    std::ifstream file1("beijing_adj.txt");
    std::ifstream file2("beijing_sent_matched_timed200.txt");

   
    // Declare ofstream object for output
    std::ofstream outputFile("output.txt");
   
// creating a template function to use pair in unordered set/map
struct hash_pair {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

void takeInput(vector<double> &double_list){  // to read input from beijing_adj file
    string double_string;
    //file1>>double_string;
    getline(file1, double_string);
    //cout<<double_string<<"\n";
    string s="";
    int commaCnt=0;
    for(int i=0;i<double_string.length();i++){
        if(double_string[i]!=','){
            s+=double_string[i];
        }
        else{
            commaCnt++;
            double_list.push_back(stold(s));
            s="";
        }
        if(commaCnt==4){
           break; 
        }
    }
    if(double_string.length()>0){
        double_list.push_back(stod(double_string));
    } 
}
void tripInput(int &src,int &dest){ // to take input from trip file
    std::string input;
    // Use std::getline to read a line of input until newline
    std::getline(file2, input);
    if(input[0]!='['){
       return; 
    }
    for(int i=1;i<input.length();i++){
        if(input[i]==','){
           break; 
        }
        src=(src*10)+(input[i]-'0');  
    }
    string s="";
    int flag=0;
    for(int i=input.length()-1;i>=0;i--){
        if(input[i]=='['){
           break; 
        }
        if(input[i]==','){
           flag++; 
        } 
        if(flag==3){
           s+=input[i]; 
        }
    }
    reverse(s.begin(),s.end());
    dest=stoi(s);
}
//calculate haversine distance for linear distance
double haversine_km(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d;
}

void preprocessing_Approach2(unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,unordered_set<pair<int,double>,hash_pair>>& adjIndegree,unordered_map<int,unordered_set<pair<int,double>,hash_pair>> &adjOutdegree,double thresholdDistance){
     for(auto it:T){
        int source=it.first.first;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
        unordered_map<int,double> distance;
        distance[source] = 0;
        // Push the source node into the priority queue
        pq.push({0, source});
        // Dijkstra's algorithm
        while(!pq.empty()){
            int u = pq.top().second; 
            if(pq.top().first > thresholdDistance){
            break;
            }
            pq.pop();
            OC[source].insert(u);     
            S[u]++;
            for(auto edge:adjIndegree[u]){
                int v = edge.first;
                double w = edge.second;
                if(distance.find(v)==distance.end()){
                   distance[v]=thresholdDistance;
                }
                if(distance[u] + w < distance[v]||distance[u]+w == thresholdDistance) { // checking the current node distance from source node 
                   distance[v] = distance[u] + w;
                   pq.push({distance[v], v});
                }
            }
        }

        int destination=it.first.second;
        while(!pq.empty()){ // to empty the priority queue
            pq.pop();
        }
        distance.clear();
        distance[destination] = 0;
        // Push the source node into the priority queue
        pq.push({0, destination});
        // Dijkstra's algorithm
        while(!pq.empty()){
            int u = pq.top().second; 
            if(pq.top().first > thresholdDistance){
            break;
            }
            pq.pop();
            EC[destination].insert(u);     
            S[u]++; 
            for(auto edge:adjOutdegree[u]) {
                int v = edge.first;
                double w = edge.second;
                if(distance.find(v)==distance.end()){
                   distance[v]=thresholdDistance;
                }
                if(distance[u] + w < distance[v]||distance[u]+w == thresholdDistance) { // checking the current node distance from source node 
                   distance[v] = distance[u] + w;
                   pq.push({distance[v], v});
                }
            }
        }
    }
}

void updatePairwiseGreedy(pair<int,int> qo,unordered_map<pair<int,int>,int,hash_pair> &HH,unordered_map<int,int> &oj,unordered_map<int,int> &ej,unordered_map<pair<int,int>,int,hash_pair> &U,unordered_map<pair<int,int>,int,hash_pair> &PU,unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<int,int> &H,unordered_map<int,int> &Ho,unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,set<pair<int,int>>> &NOC,unordered_map<int,set<pair<int,int>>> &NEC){
     for(auto it:T){
        int source=it.first.first,destination=it.first.second,trip=it.second;
        int newoj=oj[source],newej=ej[destination];
        if(OC[source].find(qo.first)!=OC[source].end() or OC[source].find(qo.second)!=OC[source].end()){
           newoj=1;
        }
        if(EC[destination].find(qo.first)!=EC[destination].end() or EC[destination].find(qo.second)!=EC[destination].end()){
           newej=1;
        }
        set<pair<int,int>>X,Y,Z; 
        for(auto it1:NOC[source]){
            if(NEC[destination].find({it1.first,it1.second})==NEC[destination].end() and NEC[destination].find({it1.second,it1.first})==NEC[destination].end())
                X.insert(it1);
        }
        
        for(auto it1:NEC[destination]){
            if(NOC[source].find({it1.second,it1.first})!=NOC[source].end() or NOC[source].find({it1.first,it1.second})!=NOC[source].end())
                Y.insert(it1);
        } 
 
        for(auto it1:NEC[destination]){
            if(NOC[source].find({it1.first,it1.second})==NOC[source].end() and NOC[source].find({it1.second,it1.first})==NOC[source].end())
                Z.insert(it1);
        }


        if((oj[source]==0 and ej[destination]==0) and (newoj==1 and newej==0)){
            for(auto ss:X){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    PU[ss]=PU[ss]-1;
                }
            }
            for(auto ss:Z){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    PU[ss]=PU[ss]-1;
                    U[ss]=U[ss]+1;
                }
            }
        }
        else if((oj[source]==0 and ej[destination]==1) and (newoj==1 and newej==1)){
            for(auto ss:NOC[source]){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    U[ss]=U[ss]-1;
                }
            }
        }
        else if((oj[source]==0 and ej[destination]==0) and (newoj==0 and newej==1)){
            for(auto ss:Z){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    PU[ss]=PU[ss]-1;
                }
            }
            for(auto ss:X){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    U[ss]=U[ss]+1;
                    PU[ss]=PU[ss]-1;
                }
            }
        }
        else if((oj[source]==1 and ej[destination]==0) and (newoj==1 and newej==1)){
            for(auto ss:NEC[destination]){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    U[ss]=U[ss]-1;
                }
            } 
        }
        else if((oj[source]==0 and ej[destination]==0) and (newoj==1 and newej==1)){
            set<pair<int,int>>X1;
            for(auto it1:X){
                if(X1.find({it1.first,it1.second})==X1.end() and X1.find({it1.second,it1.first})==X1.end())
                   X1.insert(it1);
            }
        
            for(auto it1:Z){
                if(X1.find({it1.first,it1.second})==X1.end() and X1.find({it1.second,it1.first})==X1.end())
                   X1.insert(it1);
            }
            for(auto ss:X1){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    PU[ss]=PU[ss]-1;
                }
            } 

            for(auto ss:Y){
                if(HH.find({ss.first,ss.second})==HH.end() and HH.find({ss.second,ss.first})==HH.end()){
                    U[ss]=U[ss]-1;
                }
            } 
        }
        // oj[source]=newoj;
        // ej[destination]=newej;
        // oj[source]=newoj;
        // ej[destination]=newej;
     }
     oj[qo.first]=1;
     ej[qo.first]=1;
     oj[qo.second]=1;
     ej[qo.second]=1;  
} 

void intialisePairwiseGreedy(unordered_map<int,int> &oj,unordered_map<int,int> &ej,unordered_map<pair<int,int>,int,hash_pair> &U,unordered_map<pair<int,int>,int,hash_pair> &PU,unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<int,int> &H,unordered_map<pair<int,int>,int,hash_pair> &HH,unordered_map<int,int> &Ho,unordered_map<pair<int,int>,int,hash_pair> &T,unordered_map<int,set<pair<int,int>>> &NOC,unordered_map<int,set<pair<int,int>>> &NEC){
     for(auto it:Ho){
        S[it.first]=1;
     } 
    
     for(auto it1:S){
        for(auto it2:S){
          if(U.find({it2.first,it1.first})==U.end() and U.find({it1.first,it2.first})==U.end()){ 
            if(it1.first<it2.first){
               U[{it1.first,it2.first}]=0;
               PU[{it1.first,it2.first}]=0;
            }
            else{
                U[{it2.first,it1.first}]=0;
                PU[{it2.first,it1.first}]=0;
            }   
          }
        }
     }

     for(auto it:T){
        int source=it.first.first,destination=it.first.second,trip=it.second;
        oj[source]=0;
        ej[destination]=0;

        for(auto it1:OC[source]){
            for(auto it2:S){
                if(NOC[source].find({it1,it2.first})==NOC[source].end() and NOC[source].find({it2.first,it1})==NOC[source].end())                   
                   if(it1<it2.first){
                      NOC[source].insert({it1,it2.first});
                   }
                   else{
                        NOC[source].insert({it2.first,it1});
                   }
            }
        }
        for(auto it1:EC[destination]){
            for(auto it2:S){
                if(NEC[destination].find({it1,it2.first})==NEC[destination].end() and NEC[destination].find({it2.first,it1})==NEC[destination].end())
                    if(it1<it2.first){
                       NEC[destination].insert({it1,it2.first});
                    }
                    else{
                        NEC[destination].insert({it2.first,it1});
                    }
            }
        }
        set<pair<int,int>>X,Y,Z;
        for(auto it1:NOC[source]){
            if(X.find({it1.first,it1.second})==X.end() and X.find({it1.second,it1.first})==X.end())
                X.insert(it1);
        }
        
        for(auto it1:NEC[destination]){
            if(X.find({it1.first,it1.second})==X.end() and X.find({it1.second,it1.first})==X.end())
                X.insert(it1);
        }
            
        for(auto it1:NEC[destination]){
            if(NOC[source].find({it1.second,it1.first})!=NOC[source].end() or NOC[source].find({it1.first,it1.second})!=NOC[source].end())
                Y.insert(it1);
        } 
 
        for(auto it1:X){
            if(Y.find({it1.first,it1.second})==Y.end() and Y.find({it1.second,it1.first})==Y.end())
                Z.insert(it1);
        }
        
        for(auto it:Y){
            U[it]+=1;
        }

        for(auto it:Z){
            PU[it]+=1;
        }
     }

        
        //  outputFile2<<"----------------------------------------------------------NOC\n";
        //  for(auto it:NOC){
        //     outputFile2<<it.first<<"->";
        //     for(auto it1:NOC[it.first])
        //     outputFile2<<it1.first<<" "<<it1.second<<"     ";
        //     outputFile2<<"\n";
        //  }

        //  outputFile2<<"---------------------------------------------------------------------NEC\n";
        //  for(auto it:NEC){
        //     outputFile2<<it.first<<"->";
        //     for(auto it1:NEC[it.first])
        //     outputFile2<<it1.first<<" "<<it1.second<<"     ";
        //     outputFile2<<"\n";
        //  }

        
        //  outputFile2<<"----------------------------------------------------------U\n";
        //  for(auto it:U){
        //     outputFile2<<it.first.first<<" "<<it.first.second<<" -> "<<it.second<<"\n";
        //  }

        //  outputFile2<<"---------------------------------------------------------------------PU\n";
        //  for(auto it:PU){
        //     outputFile2<<it.first.first<<" "<<it.first.second<<" -> "<<it.second<<"\n";
        //  }

     for(auto it:Ho){
        HH[{it.first,it.first}]=1;
        for(auto it1:H){
            if(HH.find({it1.first,it.first})==HH.end() and HH.find({it.first,it1.first})==HH.end()){
                if(it.first<it1.first){
                   HH[{it.first,it1.first}]=1;
                }
                else{
                    HH[{it1.first,it.first}]=0;
                }
            }
        }
        H[it.first]++;
        pair<int,int> qo={it.first,it.first};
        updatePairwiseGreedy(qo,HH,oj,ej,U,PU,S,OC,EC,H,Ho,T,NOC,NEC);
     }
}


int pairwiseGreedy(int k,unordered_map<int,int> &Ho,unordered_map<int,int> &S,unordered_map<int,unordered_set<int>> &OC,unordered_map<int,unordered_set<int>> &EC,unordered_map<pair<int,int>,int,hash_pair> &T,double thresholdDistance){
     unordered_map<int,int> H,oj,ej;
     unordered_map<pair<int,int>,int,hash_pair> U,PU,HH;
     // H -> Hub, U -> Utility, PU -> Potential Utility
     unordered_map<int,set<pair<int,int>>> NOC,NEC;
     intialisePairwiseGreedy(oj,ej,U,PU,S,OC,EC,H,HH,Ho,T,NOC,NEC);
     while(k>0){
        //  outputFile2<<"----------------------------------------------------------U\n";
        //  for(auto it:U){
        //     outputFile2<<it.first.first<<" "<<it.first.second<<" -> "<<it.second<<"\n";
        //  }

        //  outputFile2<<"---------------------------------------------------------------------PU\n";
        //  for(auto it:PU){
        //     outputFile2<<it.first.first<<" "<<it.first.second<<" -> "<<it.second<<"\n";
        //  }

        int count=0,Utility=INT_MIN;
        pair<int,int> qo;
        for(auto it1:U){
            auto it=it1.first;
            if(H.find(it.first)==H.end() and H.find(it.second)==H.end()){
               if(k==1 and (it.first!=it.second)){
                continue;
               }
               if(it1.second>Utility){
                  Utility=it1.second;
                  qo=it;
                  count=1; 
               }
               else if(it1.second==Utility){
                    count++;
                    qo=it;
               }
            }
        }
        
        unordered_map<pair<int,int>,int,hash_pair> minUtilityHub;
        if(count==0){
          minUtilityHub=U;
        }
        else if(count==1){
            HH[{qo.first,qo.second}]=1;
            for(auto it1:H){
               if(HH.find({it1.first,qo.first})==HH.end() and HH.find({qo.first,it1.first})==HH.end()){
                  if(qo.first<it1.first){
                     HH[{qo.first,it1.first}]=1;
                  }   
                  else{
                      HH[{it1.first,qo.first}]=1;
                  }    
               }
                if(HH.find({it1.first,qo.second})==HH.end() and HH.find({qo.second,it1.first})==HH.end()){
                   if(qo.second<it1.first){
                     HH[{qo.second,it1.first}]=1;
                  }   
                  else{
                      HH[{it1.first,qo.second}]=1;
                  } 
                }            
            }
            H[qo.first]=1;
            H[qo.second]=1;
            updatePairwiseGreedy(qo,HH,oj,ej,U,PU,S,OC,EC,H,Ho,T,NOC,NEC);
            if(qo.first==qo.second){
               k-=1;
            }
            else{
                k-=2;
            }
            continue;
        }
        else if(count>1){
               for(auto it:U){
                  if(it.second==Utility and (H.find(it.first.first)==H.end() and H.find(it.first.second)==H.end())){
                     minUtilityHub[it.first]++;
                  }
               }
        }
        // to find the highest potential Utility in the candidate hub location where max utility ties up
        int minTotalDegree=INT_MIN;
        for(auto it:PU){
            int totalDegree=it.second;
            if(minUtilityHub.find(it.first)!=minUtilityHub.end() and (H.find(it.first.first)==H.end() and H.find(it.first.second)==H.end())){
              if(k==1 and (it.first.first!=it.first.second)){
                continue;
               }
              if(totalDegree>minTotalDegree){
                minTotalDegree=totalDegree;
                qo=it.first; 
               }
               else if(minTotalDegree==totalDegree){
                  qo=it.first;
               }
            }
        }
        HH[{qo.first,qo.second}]=1;
            for(auto it1:H){
               if(HH.find({it1.first,qo.first})==HH.end() and HH.find({qo.first,it1.first})==HH.end()){
                  if(qo.first<it1.first){
                     HH[{qo.first,it1.first}]=1;
                  }   
                  else{
                      HH[{it1.first,qo.first}]=1;
                  }    
               }
                if(HH.find({it1.first,qo.second})==HH.end() and HH.find({qo.second,it1.first})==HH.end()){
                   if(qo.second<it1.first){
                     HH[{qo.second,it1.first}]=1;
                  }   
                  else{
                      HH[{it1.first,qo.second}]=1;
                  } 
                }            
            }
            H[qo.first]=1;
            H[qo.second]=1;
        updatePairwiseGreedy(qo,HH,oj,ej,U,PU,S,OC,EC,H,Ho,T,NOC,NEC);
        if(qo.first==qo.second){
            k-=1;
        }
        else{
            k-=2;
        }
    }

    for(auto it:H){
        if(Ho.find(it.first)==Ho.end()){
          cout<<it.first<<" ";
        }
    }
    for(auto it:Ho){
        H[it.first]++;
    }
    int totalTripCover=0;
    for(auto it:T){
        int source=it.first.first;
        int destination=it.first.second;
        int count=0;
        for(auto it1:OC[source]){
            if(H.find(it1)!=H.end()){
               count++;
               break; 
            }
        }
        for(auto it1:EC[destination]){
            if(H.find(it1)!=H.end()){
               count++;
               break; 
            }
        }
        if(count==2){
            totalTripCover+=it.second; 
        }
    }
    return totalTripCover;
}

int main(){
    // to speed up the input and output process 
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    cout << fixed << setprecision(7);
    outputFile << fixed << setprecision(7);

    // Check if the files are opened successfully
    if (!file1.is_open() || !file2.is_open() || !outputFile.is_open()) {
        std::cerr << "Error opening files" << std::endl;
        return 1; // Return an error code
    }
    
    int nodes=0,edges=0;
    vector<double> input;
    takeInput(input);
    nodes=input[0];
    edges=input[1];
    
    map<int,pair<double,double>> nodesLatLong;// map to store Latitude and Longitude of each node
    for(int i=0;i<nodes;i++){
        input.clear();
        takeInput(input); // taking input all nodes from beijing_adj file
        nodesLatLong[input[0]]={input[1],input[2]};
    }

    unordered_map<int,unordered_set<pair<int,double>,hash_pair>> adjIndegree,adjOutdegree;
    for(int i=0;i<edges;i++){
        input.clear();
        takeInput(input); // taking input all edges from beijing_adj file
        int source=input[1];
        int destination=input[2];
        double distance=haversine_km(nodesLatLong[source].first,nodesLatLong[source].second,nodesLatLong[destination].first,nodesLatLong[destination].second);
        if(input[3]==0){
           adjOutdegree[source].insert({destination,distance});
           adjOutdegree[destination].insert({source,distance});
           adjIndegree[source].insert({destination,distance});
           adjIndegree[destination].insert({source,distance}); 
        }  
        else{
            adjOutdegree[source].insert({destination,distance});
            adjIndegree[destination].insert({source,distance});
        }
    }

    unordered_map<pair<int,int>,int,hash_pair> T;// to storing end point of trip
    for(int i=0;i<320;i++){
        int src=0,dest=0;
        tripInput(src,dest); // taking input all trip end point from mapping file
        if(i%2==1){
          T[{src,dest}]++; 
        }
    }     
    double thresholdDistance=0.1;
    unordered_map<int,unordered_set<int>> OC,EC;
    unordered_map<int,int> S,Ho;
    //S -> Candiadte Hub location, Ho -> Intial Hub
    preprocessing_Approach2(S,OC,EC,T,adjIndegree,adjOutdegree,thresholdDistance);
    double time1=(float)clock() / CLOCKS_PER_SEC ;
    cerr << "time taken in input process : " << time1<< " secs" << endl;    
    int k=10;
    cout<<pairwiseGreedy(k,Ho,S,OC,EC,T,thresholdDistance); 	
    double time2=(float)clock() / CLOCKS_PER_SEC ;
    cerr << "\ntime taken to run the Algorithm : " << time2-time1 << " secs" << endl;
    return 0;
}
