// WebAssembly 版海南大学导航算法
// 编译: emcc bridge.cpp -o main.js -s EXPORTED_FUNCTIONS="['_dijkstra','_findAllPaths','_malloc','_free']" -s EXPORTED_RUNTIME_METHODS="['ccall','UTF8ToString','lengthBytesUTF8','stringToUTF8']"

#include<emscripten.h>
#include<vector>
#include<climits>
#include<string>
#include<algorithm>
#include<cstring>
using namespace std;

//景点名称
static const char* names[]={
  "第一食堂","行政楼","二号教学楼","九教","一田",
  "书院宿舍楼","图书馆","南门","东门","游泳池"
};
static const int V=10;

//邻接矩阵(步行=0,车行=1)
static int walkAdj[V][V];
static int carAdj[V][V];

//初始化路网
static void initGraph(){
  for(int i=0;i<V;i++)
    for(int j=0;j<V;j++)
      walkAdj[i][j]=carAdj[i][j]=INT_MAX;
  for(int i=0;i<V;i++)
    walkAdj[i][i]=carAdj[i][i]=0;

  int edge[25][4]={
    {0,1,3,0},{1,2,2,0},{0,3,5,0},{0,8,3,0},{2,6,3,0},
    {3,4,6,0},{4,9,3,0},{5,0,6,0},{6,7,6,0},{6,8,5,0},
    {7,8,7,0},{6,4,5,0},//步行道
    {1,2,4,1},{1,6,2,1},{1,7,10,1},{2,3,2,1},
    {3,8,7,1},{4,5,4,1},{5,9,5,1},{0,1,1,1},
    {0,7,6,1},{2,4,3,1},{3,6,3,1},{7,9,4,1}
  };

  for(int i=0;i<25;i++){
    int from=edge[i][0],to=edge[i][1],w=edge[i][2],t=edge[i][3];
    if(t==0){walkAdj[from][to]=w;walkAdj[to][from]=w;}
    else{carAdj[from][to]=w;carAdj[to][from]=w;}
  }
}

//Dijkstra:给定邻接矩阵,返回路径字符串
static string dijkstraStr(int start,int end,int(*adj)[V]){
  int dist[V],prev[V],visited[V]={0};
  for(int i=0;i<V;i++){
    dist[i]=adj[start][i];
    prev[i]=(dist[i]==INT_MAX)?-1:start;
  }
  visited[start]=1;
  prev[start]=-1;
  dist[start]=0;

  for(int i=0;i<V-1;i++){
    int u=-1,mini=INT_MAX;
    for(int j=0;j<V;j++)
      if(!visited[j]&&dist[j]<mini){mini=dist[j];u=j;}
    if(u==-1)break;
    visited[u]=1;
    for(int v=0;v<V;v++){
      if(!visited[v]&&dist[u]!=INT_MAX&&adj[u][v]!=INT_MAX
         &&dist[u]+adj[u][v]<dist[v]){
        dist[v]=dist[u]+adj[u][v];
        prev[v]=u;
      }
    }
  }

  if(dist[end]==INT_MAX)return "NO_PATH";
  //格式: "dist,idx0,idx1,...,idxN"
  string s=to_string(dist[end]);
  vector<int>path;
  int cur=end;
  while(cur!=-1){path.push_back(cur);cur=prev[cur];}
  reverse(path.begin(),path.end());
  for(int i=0;i<(int)path.size();i++)
    s+=","+to_string(path[i]);
  return s;
}

//DFS找所有路径
static string dfsAllStr(int start,int end,int(*adj)[V]){
  vector<vector<int>>allPaths;
  vector<int>path,dist;
  int visited[V]={0};

  //用栈模拟DFS(避免递归爆栈)
  struct Frame{int cur;int nextNei;int total;};
  vector<Frame>stk;
  stk.push_back({start,0,0});
  visited[start]=1;
  path.push_back(start);

  while(!stk.empty()){
    Frame& f=stk.back();
    if(f.cur==end){
      allPaths.push_back(path);
      vector<int>d;d.push_back(f.total);
      dist.push_back(f.total);
      goto backtrack;
    }
    //找下一个未访问邻居
    while(f.nextNei<V){
      if(!visited[f.nextNei]&&adj[f.cur][f.nextNei]!=INT_MAX)break;
      f.nextNei++;
    }
    if(f.nextNei>=V){
      backtrack:
      visited[path.back()]=0;
      path.pop_back();
      stk.pop_back();
      if(!stk.empty())stk.back().nextNei++;
    }else{
      int next=f.nextNei;
      int w=adj[f.cur][next];
      visited[next]=1;
      path.push_back(next);
      stk.push_back({next,0,f.total+w});
    }
  }

  //按距离排序
  for(int i=0;i<(int)allPaths.size();i++){
    for(int j=i+1;j<(int)allPaths.size();j++){
      if(dist[i]>dist[j]){
        swap(allPaths[i],allPaths[j]);
        swap(dist[i],dist[j]);
      }
    }
  }

  string s;
  for(int i=0;i<(int)allPaths.size();i++){
    if(i>0)s+="|";
    for(int j=0;j<(int)allPaths[i].size();j++){
      if(j>0)s+="-";
      s+=to_string(allPaths[i][j]);
    }
    s+=":"+to_string(dist[i]);
  }
  return s.empty()?"EMPTY":s;
}

extern "C" {
  EMSCRIPTEN_KEEPALIVE
  void init(){initGraph();}

  //dijkstra(mode,start,end) -> 返回字符串指针(NULL=不可达)
  //格式:"距离,节点0,节点1,..."
  EMSCRIPTEN_KEEPALIVE
  const char* dijkstra(int mode,int start,int end){
    initGraph();
    int(*adj)[V]=(mode==0)?walkAdj:carAdj;
    static string result;
    result=dijkstraStr(start,end,adj);
    return result.c_str();
  }

  //findAllPaths(mode,start,end) -> 所有路径字符串
  //格式:"路径1_idx-...-idx:距离|路径2...|..."
  EMSCRIPTEN_KEEPALIVE
  const char* findAllPaths(int mode,int start,int end){
    initGraph();
    int(*adj)[V]=(mode==0)?walkAdj:carAdj;
    static string result;
    result=dfsAllStr(start,end,adj);
    return result.c_str();
  }
}
